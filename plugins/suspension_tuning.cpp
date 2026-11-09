#include <atomic>
#include <array>
#include <unordered_map>
// Suspension Tuning -- a brake disc + caliper and a coilover at every wheel, as
// plugins\mt_plugin_SuspensionTuning.dll. Local visuals only: nothing replicated, no physics touched.
//
// What drives it: each MHWheelComponent (MTVehicle.Wheels) is moved by the game every frame --
// RelativeLocation carries the suspension travel, RelativeRotation the spin and the steer. The
// wheel CDO's RelativeLocation is the blueprint's own wheel position, which is the fixed reference
// the top mount hangs off: a raised (dune) build puts the live wheel further below it, so the shock
// reads longer. Steer comes off the wheel's axle direction (the Y axis of its rotation), which spin
// leaves alone -- so the brake unit steers with the wheel but never turns with it.
//
// The coilover grows from the wheel's socket -- the blueprint's own wheel position, fixed on the
// body -- a little inboard and above it, down to the knuckle at the tyre's inner face. It is the
// Suspension attachment's mesh (PolygonStreetRacer SM_Prop_Suspension_01) stretched to that length,
// so ride height, spacers and wheel width lengthen and lean it, and it never rises past the arch.
// Below it a knuckle carries the disc, a lower arm runs from the knuckle to a pivot fixed at the
// socket, and a driven wheel (one with a differential or drive shaft) gets a half-shaft to the
// body's middle -- so the brake reads as part of the car. Arini's SM_Coilover (Spring/Damper slots),
// SM_Knuckle, SM_HalfShaft and SM_ControlArm (/Game/SuspensionVisuals); BasicShapes without them.
// Paint: a fitted brake kit lists Brake Caliper, Brake Disc, Suspension and Dampers on the game's
// own Paint screen (its ColorSlots); the colours come back from Customization.BodyColors by slot name.
// All pieces are added to the vehicle at runtime and attached to the wheel's own parent, so their
// relative transforms are in the same space as the wheel's, and re-posed within ~1 ms of the game
// moving the wheel (a thread of its own queues the tick every millisecond; an unmoved wheel costs a compare).
//
// Cost: only the MaxVehicles nearest the camera within Range metres are decorated, one new vehicle
// per frame at most. Components are made once per vehicle and destroyed when it leaves the range.
// A piece is moved only when its wheel moved or steered. One tick queued at a time, no per-frame
// allocation or logging. A fault in the tick pauses it, five stop it for the session.
//
// Settings: plugins\suspension_tuning.ini [General] Disabled, [SuspensionTuning] Range (60 m),
// MaxVehicles (8), BrakeInset (0.25: how far the disc sits inboard of the tyre centre, in tyre widths).
//
// shortcut: vehicles with fewer than three wheels (bikes) are skipped, and a dual-wheel pair gets
// one set on the inner wheel. Solid axles still get a coilover per wheel.
// The brake unit is Arini's disc + caliper cut from a pack wheel (/Game/SuspensionVisuals, in the MoreTuning pak);
// without them, primitives.
// shortcut: which end of the prop is the spring is a guess; [SuspensionTuning] FlipShock=1 turns it.

#include <algorithm>
#include <cmath>
#include <map>
#include <mutex>
#include <string>
#include <vector>
#include "../include/mt_plugin.hpp"
#include <timeapi.h>
#pragma comment(lib, "winmm.lib")

using namespace mt;

struct Ref { int index; uintptr_t ptr; };
static bool alive(const Ref& r) { return r.ptr && api->object(r.index) == r.ptr; }
static Ref refOf(uintptr_t o) { return {o ? get<int>(o + L().obj_index) : 0, o}; }
static bool usable(const Ref& r) { return alive(r) && !(get<uint32_t>(r.ptr + L().obj_flags) & 0x18000); }  // not being destroyed

struct FStr { const wchar_t* data; int num, max; };
static FStr str(const wchar_t* s) { int n = (int)wcslen(s) + 1; return {s, n, n}; }

// ---- math (UE: X forward, Y right, Z up; FQuat x y z w) ----
struct V { double x, y, z; };
struct Q { double x, y, z, w; };
static V add(V a, V b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
static V sub(V a, V b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
static V mul(V a, double k) { return {a.x * k, a.y * k, a.z * k}; }
static double dot(V a, V b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static V cross(V a, V b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
static double len(V a) { return sqrt(dot(a, a)); }
static Q qmul(Q a, Q b) {
    return {a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y, a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w, a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}
static V rot(Q q, V v) {
    V u{q.x, q.y, q.z}, t = mul(cross(u, v), 2);
    return add(add(v, mul(t, q.w)), cross(u, t));
}
static Q axisAngle(V a, double r) { double s = sin(r / 2); return {a.x * s, a.y * s, a.z * s, cos(r / 2)}; }
static Q fromZ(V u) {  // the rotation taking +Z onto unit u
    V ax = cross({0, 0, 1}, u);
    double s = len(ax), c = u.z;
    if (s < 1e-9) return c > 0 ? Q{0, 0, 0, 1} : Q{1, 0, 0, 0};
    return axisAngle(mul(ax, 1 / s), atan2(s, c));
}
static Q fromBasis(V x, V y, V z) {  // the rotation taking X, Y, Z onto these unit axes
    double t = x.x + y.y + z.z;
    if (t > 0) { double r = sqrt(1 + t) * 2; return {(y.z - z.y) / r, (z.x - x.z) / r, (x.y - y.x) / r, r / 4}; }
    if (x.x > y.y && x.x > z.z) { double r = sqrt(1 + x.x - y.y - z.z) * 2; return {r / 4, (y.x + x.y) / r, (z.x + x.z) / r, (y.z - z.y) / r}; }
    if (y.y > z.z) { double r = sqrt(1 + y.y - x.x - z.z) * 2; return {(y.x + x.y) / r, r / 4, (z.y + y.z) / r, (z.x - x.z) / r}; }
    double r = sqrt(1 + z.z - x.x - y.y) * 2; return {(z.x + x.z) / r, (z.y + y.z) / r, r / 4, (x.y - y.x) / r};
}
static Q along(V u, V ref) {  // Z on u, X square to u and ref (so a flat piece lies with ref through its thickness)
    V x = cross(ref, u);
    if (len(x) < 1e-6) x = cross(V{1, 0, 0}, u);
    x = mul(x, 1 / len(x));
    return fromBasis(x, cross(u, x), u);
}
static Q yAlong(V a) {  // Y on unit a, Z as near up as it can be (a flat piece lies flat)
    V x = cross(a, V{0, 0, 1});
    if (len(x) < 1e-6) x = V{1, 0, 0};
    x = mul(x, 1 / len(x));
    return fromBasis(x, a, cross(x, a));
}
static const double PI = 3.14159265358979;

// ---- engine ----
static uintptr_t cVeh, cWheel, cMesh, fAdd, fFinish, fSetMesh, fMobility, fCollision, fShadow, fSetMaterial, fXf, fAttach, fDestroy;
static uintptr_t fMeshBounds, fLoc, fCompLoc, fCamMgr, fCamLoc, gameplayCDO, fMakeMid, matCDO, fMidColor, fToName, stringCDO;
static uintptr_t fPath, fRef, fLoad, systemCDO, fRendered;
static int oParts = -1, oPartKey = -1, oPartSlot = -1, partSize;
static int oId = -1;  // MTVehicle.Net_VehicleId: the key of a car's Tuning settings
static bool kPaintSlots = true, kDirt = true, kSpin = true;  // [SuspensionTuning] PaintSlots / Dirt / Spin
static int kSettleMs = 10000;                                // [SuspensionTuning] SettleSeconds
static const char* volatile gStep = "-";  // what the tick is doing, for a fault's log line
static int oBodyColors = -1, oColSlot, oColColor, colSize, oDiffComp = -1, oShaftComp = -1;  // paint, and whether a wheel is driven
static const int SLOT_BRAKE_POWER = 202;  // EMTVehiclePartSlot::BrakePower (usmap 720T6)
static int oWheels, oRelLoc, oRelRot, oRelScale, oParent, oRoot, oStaticMesh, pXf, pXfTeleport;
static int kRange, kMax;
static double kInset;
static bool kFlip;

static uintptr_t loadAsset(const wchar_t* path) {
    unsigned char a[0x80] = {0}, b[0x80] = {0}, c[0x80] = {0};
    FStr s = str(path);
    int in = off(fRef, "SoftObjectPath"), load = off(fLoad, "Asset"), got = off(fLoad, "ReturnValue");
    memcpy(a + off(fPath, "PathString"), &s, sizeof s);
    api->process_event(systemCDO, fPath, a);
    memcpy(b + in, a + off(fPath, "ReturnValue"), off(fRef, "ReturnValue") - in);
    api->process_event(systemCDO, fRef, b);
    memcpy(c + load, b + off(fRef, "ReturnValue"), got - load);
    api->process_event(systemCDO, fLoad, c);
    uintptr_t out = 0;
    memcpy(&out, c + got, 8);
    return out;
}

// A mesh's local box: centre and full size.
struct Box { V centre, size; };
static Box boundsOf(uintptr_t mesh) {
    unsigned char b[0x40] = {0};
    api->process_event(mesh, fMeshBounds, b);
    V o, e; memcpy(&o, b, 24); memcpy(&e, b + 24, 24);
    return {o, mul(e, 2)};
}

// ---- assets: kept by Ref, reloaded when the GC took them (nothing holds them with no car decorated) ----
// Calipers: the kit's colour (MoreTuning's C0001_BBK<mm>_<Colour> BrakePower rows), else grey.
enum { M_DISC, M_BODY, M_GREY, M_RED, M_YELLOW, M_BLUE, M_BLACK, M_SILVER, M_GOLD, NMAT };
static const float COLOURS[NMAT][3] = {{0.06f, 0.06f, 0.065f}, {0.03f, 0.03f, 0.035f}, {0.30f, 0.30f, 0.32f}, {0.55f, 0.02f, 0.01f}, {0.85f, 0.60f, 0.02f},
                                       {0.02f, 0.10f, 0.55f}, {0.02f, 0.02f, 0.02f}, {0.70f, 0.71f, 0.73f}, {0.75f, 0.52f, 0.12f}};
static const char* const KIT_COLOURS[] = {"Red", "Yellow", "Blue", "Black", "Silver", "Gold"};  // M_RED.. in order
static Ref cylinder, cube, shock, disc, caliper, mats[NMAT];
static Ref coil, knuckle, halfShaft, arm, kHub, kWeb, kMounts, spacer, kArm, kEye, kBall, spBody, spLugs, shStub, cvBoot, cvInner, cvBall, cvRubber, coBot, coTop, coBody, coSpring, dBody, dRod, armBall, armLink, armBush;  // the coilover split: rigid eyes, stretching middle  // kEye/kBall: the mounts split, each moved on its own  // kArm: the two rods hub -> mounts, in place of the web  // kHub/kWeb/kMounts: the knuckle split so only the web stretches  // Arini's suspension set (/Game/SuspensionVisuals)
static bool suspMesh;
static bool brakeMesh;  // Arini's disc + caliper (cut from a pack wheel), painted through M_Body
static Box cylBox, cubeBox, shockBox;
static Q shockPre;  // turns the prop's long axis onto +Z
static uint64_t colorName;

// A material of our own in one colour (linear RGB): M_Body under Arini's meshes, else BasicShapeMaterial.
static Ref newMid(uintptr_t world, const float rgb[3]) {
    uintptr_t base = brakeMesh ? loadAsset(L"/Game/Cars/Materials/Body/M_Body.M_Body") : 0;
    if (!base) base = loadAsset(L"/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial");
    if (!base) return {};
    unsigned char p[0x40] = {0};
    memcpy(p + off(fMakeMid, "WorldContextObject"), &world, 8);
    memcpy(p + off(fMakeMid, "Parent"), &base, 8);
    api->process_event(matCDO, fMakeMid, p);
    uintptr_t mid = 0;
    memcpy(&mid, p + off(fMakeMid, "ReturnValue"), 8);
    if (mid) {
        struct { uint64_t name; float rgba[4]; } c = {colorName, {rgb[0], rgb[1], rgb[2], 1}};
        api->process_event(mid, fMidColor, &c);
    }
    return refOf(mid);
}
static void setColour(const Ref& mid, uint32_t bgra) {  // the Paint screen's sRGB FColor, as linear
    float rgb[3] = {(float)pow(((bgra >> 16) & 255) / 255.0, 2.2), (float)pow(((bgra >> 8) & 255) / 255.0, 2.2), (float)pow((bgra & 255) / 255.0, 2.2)};
    struct { uint64_t name; float rgba[4]; } c = {colorName, {rgb[0], rgb[1], rgb[2], 1}};
    if (usable(mid)) api->process_event(mid.ptr, fMidColor, &c);
}
// ---- dirt (0.7.20): M_Body draws it from Dirt* parameters the game's MTVehicleDirtComponent sets on each car
// material (state texture, lookups, the wheel's sample offset, the wheel-uniform mask). Our pieces are wheel parts,
// so each corner's materials take every Dirt* parameter of its own wheel's material: the same dirt as that wheel.
static int oSclVals = -2, oVecVals, oTexVals, sclSize, vecSize, texSize, oInfoName, oSclVal, oVecVal, oTexVal;
static uintptr_t cMatInst, fMidScalar, fMidTex, fGetMat;
static bool dirtReady() {
    if (oSclVals != -2) return oSclVals >= 0;
    oSclVals = -1;
    cMatInst = api->find_object("Class", "MaterialInstance", "/Script/Engine");
    uintptr_t sScl = api->find_object("ScriptStruct", "ScalarParameterValue", "/Script/Engine"),
              sVec = api->find_object("ScriptStruct", "VectorParameterValue", "/Script/Engine"),
              sTex = api->find_object("ScriptStruct", "TextureParameterValue", "/Script/Engine"),
              sInfo = api->find_object("ScriptStruct", "MaterialParameterInfo", "/Script/Engine");
    fMidScalar = api->find_object("Function", "SetScalarParameterValue", "MaterialInstanceDynamic");
    fMidTex = api->find_object("Function", "SetTextureParameterValue", "MaterialInstanceDynamic");
    fGetMat = api->find_object("Function", "GetMaterial", "PrimitiveComponent");
    if (!cMatInst || !sScl || !sVec || !sTex || !sInfo || !fMidScalar || !fMidTex || !fGetMat) { MT_LOG("dirt: material layout not found -- pieces stay clean"); return false; }
    int vals[3] = {off(cMatInst, "ScalarParameterValues"), off(cMatInst, "VectorParameterValues"), off(cMatInst, "TextureParameterValues")};
    int info = off(sScl, "ParameterInfo");
    oInfoName = off(sInfo, "Name");
    oSclVal = off(sScl, "ParameterValue"); oVecVal = off(sVec, "ParameterValue"); oTexVal = off(sTex, "ParameterValue");
    sclSize = get<int>(sScl + L().struct_child_props + 8); vecSize = get<int>(sVec + L().struct_child_props + 8); texSize = get<int>(sTex + L().struct_child_props + 8);
    if (vals[0] < 0 || vals[1] < 0 || vals[2] < 0 || info != 0 || oInfoName != 0 || oSclVal < 0 || oVecVal < 0 || oTexVal < 0 || sclSize <= 0 || vecSize <= 0 || texSize <= 0) {
        MT_LOG("dirt: parameter layout not as expected -- pieces stay clean"); return false;
    }
    oVecVals = vals[1]; oTexVals = vals[2]; oSclVals = vals[0];
    return true;
}
static bool isDirtName(uintptr_t at) { char b[64]; return api->fname(at, b, sizeof b) && !strncmp(b, "Dirt", 4); }
// The wheel's first material that carries the dirt state texture.
static uintptr_t dirtSource(uintptr_t wheel) {
    for (int slot = 0; slot < 6; slot++) {
        struct { int index; uintptr_t ret; } q = {slot, 0};
        api->process_event(wheel, fGetMat, &q);
        if (!q.ret || !api->is_a(classOf(q.ret), cMatInst)) continue;
        uintptr_t d = get<uintptr_t>(q.ret + oTexVals);
        int n = get<int>(q.ret + oTexVals + 8);
        for (int i = 0; d && i < n && i < 64; i++)
            if (isDirtName(d + (uintptr_t)i * texSize + oInfoName) && get<uintptr_t>(d + (uintptr_t)i * texSize + oTexVal)) return q.ret;
    }
    return 0;
}
static int copyDirt(uintptr_t from, const Ref& to) {
    if (!from || !usable(to)) return 0;
    int done = 0;
    uintptr_t d = get<uintptr_t>(from + oSclVals); int n = get<int>(from + oSclVals + 8);
    for (int i = 0; d && i < n && i < 128; i++) {
        uintptr_t e = d + (uintptr_t)i * sclSize;
        if (!isDirtName(e + oInfoName)) continue;
        struct { uint64_t name; float v; } q = {get<uint64_t>(e + oInfoName), get<float>(e + oSclVal)};
        api->process_event(to.ptr, fMidScalar, &q); done++;
    }
    d = get<uintptr_t>(from + oVecVals); n = get<int>(from + oVecVals + 8);
    for (int i = 0; d && i < n && i < 128; i++) {
        uintptr_t e = d + (uintptr_t)i * vecSize;
        if (!isDirtName(e + oInfoName)) continue;
        struct { uint64_t name; float c[4]; } q = {get<uint64_t>(e + oInfoName), {}};
        rd(e + oVecVal, q.c, sizeof q.c);
        api->process_event(to.ptr, fMidColor, &q); done++;
    }
    d = get<uintptr_t>(from + oTexVals); n = get<int>(from + oTexVals + 8);
    for (int i = 0; d && i < n && i < 128; i++) {
        uintptr_t e = d + (uintptr_t)i * texSize;
        if (!isDirtName(e + oInfoName)) continue;
        struct { uint64_t name; uintptr_t t; } q = {get<uint64_t>(e + oInfoName), get<uintptr_t>(e + oTexVal)};
        if (q.t) { api->process_event(to.ptr, fMidTex, &q); done++; }
    }
    return done;
}
static void setMat(const Ref& piece, const Ref& m, int slot = 0) {
    if (!usable(piece) || !usable(m)) return;
    unsigned char q[0x20] = {0};
    memcpy(q + off(fSetMaterial, "ElementIndex"), &slot, 4);
    memcpy(q + off(fSetMaterial, "Material"), &m.ptr, 8);
    api->process_event(piece.ptr, fSetMaterial, q);
}

static bool assets(uintptr_t world) {
    if (!usable(cylinder)) { cylinder = refOf(loadAsset(L"/Engine/BasicShapes/Cylinder.Cylinder")); if (cylinder.ptr) cylBox = boundsOf(cylinder.ptr); }
    if (!usable(cube)) { cube = refOf(loadAsset(L"/Engine/BasicShapes/Cube.Cube")); if (cube.ptr) cubeBox = boundsOf(cube.ptr); }
    if (!cylinder.ptr || !cube.ptr || cylBox.size.z <= 0 || cubeBox.size.z <= 0) return false;
    if (!usable(shock)) {
        shock = refOf(loadAsset(L"/Game/Models/PolygonStreetRacer/Meshes/Props/SM_Prop_Suspension_01.SM_Prop_Suspension_01"));
        if (shock.ptr) {
            shockBox = boundsOf(shock.ptr);
            V z = shockBox.size;
            shockPre = z.x >= z.y && z.x >= z.z ? axisAngle({0, 1, 0}, -PI / 2) : z.y >= z.z ? axisAngle({1, 0, 0}, PI / 2) : Q{0, 0, 0, 1};
            if (kFlip) shockPre = qmul(axisAngle({1, 0, 0}, PI), shockPre);
            static bool told;
            if (!told) { told = true; MT_LOG("shock mesh %.1f x %.1f x %.1f cm", z.x, z.y, z.z); }
        } else MT_LOG("shock mesh not found -- plain cylinders instead");
    }
    if (!colorName) {
        unsigned char n[0x40] = {0};
        FStr name = str(L"Color");
        memcpy(n + off(fToName, "InString"), &name, sizeof name);
        api->process_event(stringCDO, fToName, n);
        memcpy(&colorName, n + off(fToName, "ReturnValue"), 8);
    }
    if (!usable(disc)) disc = refOf(loadAsset(L"/Game/SuspensionVisuals/SM_BrakeDisc.SM_BrakeDisc"));
    if (!usable(caliper)) caliper = refOf(loadAsset(L"/Game/SuspensionVisuals/SM_BrakeCaliper.SM_BrakeCaliper"));
    if (!usable(coil)) coil = refOf(loadAsset(L"/Game/SuspensionVisuals/SM_Coilover.SM_Coilover"));
    if (!usable(knuckle)) knuckle = refOf(loadAsset(L"/Game/SuspensionVisuals/SM_Knuckle.SM_Knuckle"));
    if (!usable(halfShaft)) halfShaft = refOf(loadAsset(L"/Game/SuspensionVisuals/SM_HalfShaft.SM_HalfShaft"));
    if (!usable(arm)) arm = refOf(loadAsset(L"/Game/SuspensionVisuals/SM_ControlArm.SM_ControlArm"));
    if (!usable(kHub)) kHub = refOf(loadAsset(L"/Game/SuspensionVisuals/SM_KnuckleHub.SM_KnuckleHub"));
    if (!usable(kWeb)) kWeb = refOf(loadAsset(L"/Game/SuspensionVisuals/SM_KnuckleWeb.SM_KnuckleWeb"));
    if (!usable(kMounts)) kMounts = refOf(loadAsset(L"/Game/SuspensionVisuals/SM_KnuckleMounts.SM_KnuckleMounts"));
    if (!usable(spBody)) spBody = refOf(loadAsset(L"/Game/SuspensionVisuals/SM_BrakeSpacerBody.SM_BrakeSpacerBody"));
    if (!usable(spLugs)) spLugs = refOf(loadAsset(L"/Game/SuspensionVisuals/SM_BrakeSpacerLugs.SM_BrakeSpacerLugs"));
    if (!usable(armBall)) armBall = refOf(loadAsset(L"/Game/SuspensionVisuals/SM_ArmBall.SM_ArmBall"));
    if (!usable(armLink)) armLink = refOf(loadAsset(L"/Game/SuspensionVisuals/SM_ArmLink.SM_ArmLink"));
    if (!usable(armBush)) armBush = refOf(loadAsset(L"/Game/SuspensionVisuals/SM_ArmBushing.SM_ArmBushing"));
    if (!usable(coSpring)) coSpring = refOf(loadAsset(L"/Game/SuspensionVisuals/SM_CoiloverSpring.SM_CoiloverSpring"));
    if (!usable(dBody)) dBody = refOf(loadAsset(L"/Game/SuspensionVisuals/SM_DamperBody.SM_DamperBody"));
    if (!usable(dRod)) dRod = refOf(loadAsset(L"/Game/SuspensionVisuals/SM_DamperRod.SM_DamperRod"));
    if (!usable(coBot)) coBot = refOf(loadAsset(L"/Game/SuspensionVisuals/SM_CoiloverBottom.SM_CoiloverBottom"));
    if (!usable(coTop)) coTop = refOf(loadAsset(L"/Game/SuspensionVisuals/SM_CoiloverTop.SM_CoiloverTop"));
    if (!usable(coBody)) coBody = refOf(loadAsset(L"/Game/SuspensionVisuals/SM_CoiloverBody.SM_CoiloverBody"));
    if (!usable(shStub)) shStub = refOf(loadAsset(L"/Game/SuspensionVisuals/SM_ShaftStub.SM_ShaftStub"));
    if (!usable(cvBoot)) cvBoot = refOf(loadAsset(L"/Game/SuspensionVisuals/SM_CVBoot.SM_CVBoot"));
    if (!usable(cvBall)) cvBall = refOf(loadAsset(L"/Game/SuspensionVisuals/SM_CVBall.SM_CVBall"));
    if (!usable(cvRubber)) cvRubber = refOf(loadAsset(L"/Game/SuspensionVisuals/SM_CVBootRubber.SM_CVBootRubber"));
    if (!usable(cvInner)) cvInner = refOf(loadAsset(L"/Game/SuspensionVisuals/SM_CVInner.SM_CVInner"));
    if (!usable(kEye)) kEye = refOf(loadAsset(L"/Game/SuspensionVisuals/SM_KnuckleEye.SM_KnuckleEye"));
    if (!usable(kBall)) kBall = refOf(loadAsset(L"/Game/SuspensionVisuals/SM_KnuckleBall.SM_KnuckleBall"));
    if (!usable(kArm)) kArm = refOf(loadAsset(L"/Game/SuspensionVisuals/SM_KnuckleArm.SM_KnuckleArm"));
    if (!usable(spacer)) spacer = refOf(loadAsset(L"/Game/SuspensionVisuals/SM_BrakeSpacer.SM_BrakeSpacer"));
    bool hadSusp = suspMesh;
    suspMesh = coil.ptr && knuckle.ptr && halfShaft.ptr && arm.ptr;
    static bool toldSusp, toldBrake;  // log the first answer too: a missing pak used to be silent
    if (suspMesh != hadSusp || !toldSusp) { toldSusp = true; MT_LOG(suspMesh ? "suspension: coilover, knuckle, half-shaft and arm meshes" : "suspension meshes not found (is qxZap_SuspensionVisuals_P enabled under Mods?) -- plain shapes instead"); }
    bool had = brakeMesh;
    brakeMesh = disc.ptr && caliper.ptr;
    if (brakeMesh != had || !toldBrake) {
        toldBrake = true;
        MT_LOG(brakeMesh ? "brake unit: disc + caliper meshes" : "brake unit meshes not found -- plain shapes instead");
        for (auto& m : mats) m = {};  // their parent material changes with it
    }
    for (int i = 0; i < NMAT; i++) {
        if (usable(mats[i])) continue;
        mats[i] = newMid(world, COLOURS[i]);
        if (!mats[i].ptr) return false;
    }
    return true;
}

static Ref makePiece(uintptr_t actor, uintptr_t parent, uintptr_t mesh, int mat) {
    alignas(16) unsigned char p[0x100] = {0};
    int at = off(fAdd, "RelativeTransform");
    double spin[4] = {0, 0, 0, 1}, scale[3] = {0.01, 0.01, 0.01};
    memcpy(p + off(fAdd, "Class"), &cMesh, 8);
    memcpy(p + at, spin, sizeof spin);
    memcpy(p + at + 0x40, scale, sizeof scale);
    p[off(fAdd, "bDeferredFinish")] = 1;
    api->process_event(actor, fAdd, p);
    uintptr_t c = 0;
    memcpy(&c, p + off(fAdd, "ReturnValue"), 8);
    if (!c) return {};
    uint8_t movable = 2, none = 0;
    api->process_event(c, fMobility, &movable);
    api->process_event(c, fCollision, &none);
    api->process_event(c, fShadow, &none);
    unsigned char m[0x10] = {0};
    memcpy(m, &mesh, 8);
    api->process_event(c, fSetMesh, m);
    if (mat >= 0) {  // else the mesh keeps its own
        unsigned char q[0x20] = {0};
        memcpy(q + off(fSetMaterial, "Material"), &mats[mat].ptr, 8);
        api->process_event(c, fSetMaterial, q);
    }
    neverCull(c);  // the maps' cull-distance volumes dropped these small meshes a little way off
    alignas(16) unsigned char f[0x100] = {0};
    memcpy(f + off(fFinish, "Component"), &c, 8);
    memcpy(f + off(fFinish, "RelativeTransform"), p + at, 0x60);
    api->process_event(actor, fFinish, f);
    if (parent && parent != get<uintptr_t>(actor + oRoot)) {  // into the wheel's space: KeepRelative x3
        unsigned char a[0x40] = {0};
        memcpy(a + off(fAttach, "Parent"), &parent, 8);
        api->process_event(c, fAttach, a);
    }
    return refOf(c);
}

// Puts a mesh whose local box is `b` so that box is centred at `at`, turned by q, `size` big.
static void put(const Ref& c, const Box& b, V at, Q q, V size) {
    if (!usable(c)) return;
    alignas(16) static unsigned char p[0x400];
    memset(p, 0, sizeof p);
    V s{size.x / b.size.x, size.y / b.size.y, size.z / b.size.z};
    V t = sub(at, rot(q, V{b.centre.x * s.x, b.centre.y * s.y, b.centre.z * s.z}));
    double xf[12] = {q.x, q.y, q.z, q.w, t.x, t.y, t.z, 0, s.x, s.y, s.z, 0};
    memcpy(p + pXf, xf, sizeof xf);
    p[pXfTeleport] = 1;
    api->process_event(c.ptr, fXf, p);
}

// Puts a mesh with its own origin at `at`, turned by q, scaled by s (no box fitting).
static void place(const Ref& c, V at, Q q, V s) {
    if (!usable(c)) return;
    alignas(16) static unsigned char p[0x400];
    memset(p, 0, sizeof p);
    double xf[12] = {q.x, q.y, q.z, q.w, at.x, at.y, at.z, 0, s.x, s.y, s.z, 0};
    memcpy(p + pXf, xf, sizeof xf);
    p[pXfTeleport] = 1;
    api->process_event(c.ptr, fXf, p);
}

// ---- one wheel's set ----
enum { P_DISC, P_CALIPER, P_SHOCK, P_KNUCKLE, P_ARM, P_SHAFT, P_WEB, P_MOUNTS, P_SPACER, P_WEB2, P_MOUNTS2, P_SPLUGS, P_SHAFT2, P_BOOT, P_CBOT, P_CTOP, P_LINK1, P_LINK2, P_BUSH1, P_BUSH2, P_CVIN, P_BOOTW, P_BOOTC, P_BOOTI, P_DBODY, P_DROD, NPIECE };
// Paint: the brake kits' ColorSlots (BrakeKits/run.py), stored by the game's Paint screen in Customization.BodyColors.
enum { PC_CALIPER, PC_DISC, PC_SUSP, PC_DAMPER, PC_SPACER, PC_LUGS, PC_ARMS, PC_SHAFT, NPAINT };
static const char* const PAINT_SLOTS[NPAINT] = {"QX_BrakeCaliper", "QX_BrakeDisc", "QX_Suspension", "QX_Damper", "QX_Spacer", "QX_Lugs", "QX_Arms", "QX_Shaft"};
static const wchar_t* const PAINT_WIDE[NPAINT] = {L"QX_BrakeCaliper", L"QX_BrakeDisc", L"QX_Suspension", L"QX_Damper", L"QX_Spacer", L"QX_Lugs", L"QX_Arms", L"QX_Shaft"};
static int paintName[NPAINT];  // their pool indices
// The names made by the game (Conv_StringToName), not looked up: nothing in the tables registers them since the kits
// lost their ColorSlots, and a lookup only found them on a save that already carried them.
static int paintIndex(int i) {
    if (!paintName[i] && fToName && stringCDO) {
        unsigned char n[0x40] = {0};
        FStr t = str(PAINT_WIDE[i]);
        memcpy(n + off(fToName, "InString"), &t, sizeof t);
        api->process_event(stringCDO, fToName, n);
        paintName[i] = get<int>((uintptr_t)n + off(fToName, "ReturnValue"));
    }
    return paintName[i];
}

// The Paint screen greys out a colour entry no mesh on the car carries as a material slot (2026-11-09: our four were
// listed, greyed, never painted). So our meshes' slots take the QX_* names, in memory: they are our own assets.
// FStaticMaterial: MaterialInterface @0, MaterialSlotName @8, 48 bytes.
static void nameMeshSlots() {
    static int oMats = -2;
    if (oMats == -2) { uintptr_t c = api->find_object("Class", "StaticMesh", "/Script/Engine"); oMats = c ? off(c, "StaticMaterials") : -1; }
    if (oMats < 0) return;
    struct { const Ref* mesh; int slot, paint; } map[] = {
        {&caliper, 0, PC_CALIPER}, {&disc, 0, PC_DISC}, {&disc, 1, PC_DISC}, {&disc, 2, PC_LUGS}, {&spacer, 0, PC_SPACER}, {&spacer, 1, PC_LUGS},
        {&coil, 0, PC_SUSP}, {&coil, 1, PC_DAMPER}, {&kHub, 0, PC_ARMS}, {&kArm, 0, PC_ARMS}, {&kMounts, 0, PC_ARMS},
        {&knuckle, 0, PC_ARMS}, {&kWeb, 0, PC_ARMS}, {&arm, 0, PC_ARMS}, {&halfShaft, 0, PC_SHAFT}, {&kEye, 0, PC_ARMS}, {&kBall, 0, PC_ARMS}, {&spBody, 0, PC_SPACER}, {&spLugs, 0, PC_LUGS},
        {&shStub, 0, PC_SHAFT}, {&cvBoot, 1, PC_SHAFT}, {&cvInner, 1, PC_SHAFT}, {&cvBall, 0, PC_SHAFT}, {&cvRubber, 1, PC_SHAFT},
        {&coBody, 0, PC_SUSP}, {&coBody, 1, PC_DAMPER}, {&coBot, 0, PC_DAMPER}, {&coTop, 0, PC_DAMPER},
        {&coSpring, 0, PC_SUSP}, {&dBody, 0, PC_DAMPER}, {&dRod, 0, PC_DAMPER},
        {&armBall, 0, PC_ARMS}, {&armLink, 0, PC_ARMS}, {&armBush, 0, PC_ARMS}};  // the bushing's slot 1 is rubber  // the boot's slot 0 is rubber: never painted
    static int told;
    int named = 0;
    for (auto& m : map) {
        if (!usable(*m.mesh) || !paintName[m.paint]) continue;
        uintptr_t data = get<uintptr_t>(m.mesh->ptr + oMats);
        int n = get<int>(m.mesh->ptr + oMats + 8);
        if (!data || m.slot >= n || n > 16) continue;
        uintptr_t nm = data + (uintptr_t)m.slot * 48 + 8;
        uint32_t want[2] = {(uint32_t)paintName[m.paint], 0};
        if (get<uint32_t>(nm) != want[0] || get<uint32_t>(nm + 4)) { api->write(nm, want, 8); named++; }
    }
    if (named && told++ < 4) MT_LOG("paint: %d mesh slots named for the Paint screen", named);
}
struct Corner {
    Ref wheel, piece[NPIECE];
    double spacer = 0;  // cm the game's wheel spacer pushes this wheel out
    V rest;         // the blueprint's wheel position
    double side;    // +1 right of the centreline, -1 left
    V lastLoc, lastScale; double lastSteer;
    // from the wheel mesh and its scale
    double R, W, cIn, eIn;
    bool prop;      // the shock is the attachment's mesh, not a cylinder
    bool driven;    // has a differential or drive shaft: gets a half-shaft
    Ref mid[8];     // this corner's own material per paint entry: its wheel's dirt differs from the others'
    DWORD dirtAt = 0;
    bool haveStraight = false; double straightX = 0, straightY = 0;  // the hub's x, y while not steered
    double spin = 0, lastSpin = 0;  // the wheel's roll angle about its axle (rad)
    double damperZ = 0;  // the damper's length scale, fixed once (0 = not yet): it telescopes, never squeezes
    double linkRun[2] = {0, 0}, shaftRun = 0;  // lower arm links and shaft tube, fixed once: rigid, they swing and plunge
    int axle;       // 0 = the front axle, counted back: the Tuning section's "Axle 1, 2, ..."
};
struct Car {
    Ref veh; std::vector<Corner> corners; long long id = 0;
    int axles = 0; bool axleDriven[8] = {};
    DWORD bornAt = 0;  // when it was decorated: its paint maps are left alone for a while
    DWORD resentAt = 0; int kitMm = 0, kitMat = M_GREY; std::string kit = "?";
    uint32_t paint[NPAINT] = {};  // FColor as painted, 0 = not painted
    Ref mid[NPAINT];              // this car's own material per painted slot
};

// ---- Tuning > Brakes & Suspension: where the pieces sit, per car and axle (2026-11-09, the user: "so the whole
// thing fits nicely" on a supercar's inboard suspension as well as a truck's). Stored in suspension_tuning.ini,
// [Vehicle<id>] Front.<key> / Rear.<key>, as spin-box steps.
enum { F_AXLE, F_BRAKE, F_SPACER, F_UPIN, F_UPUP, F_LOWIN, F_LOWUP, F_TOPIN, F_TOPUP, F_TOPFWD, F_SIZE, F_ARMIN, F_ARMUP, F_ARMSPREAD, F_CVOFF, F_SPSIZE, NFIT };
static const int MAXAXLE = 8;
static const struct { const wchar_t* label; const wchar_t* key; int top, def; double zero, step; const wchar_t* unit; } FIT[NFIT] = {
    {L"Axle (* driven)", L"Axle", MAXAXLE - 1, 0, 0, 1, nullptr},
    {L"Brake position (out)", L"Brake", 400, 200, -100, 0.5, L"cm"},
    {L"Spacer length", L"Spacer", 400, 200, -100, 0.5, L"cm"},
    {L"Upper mount inboard", L"UpIn", 400, 200, -100, 0.5, L"cm"},
    {L"Upper mount height", L"UpUp", 400, 200, -100, 0.5, L"cm"},
    {L"Lower mount inboard", L"LowIn", 400, 200, -100, 0.5, L"cm"},
    {L"Lower mount height", L"LowUp", 400, 200, -100, 0.5, L"cm"},
    {L"Coilover top inboard", L"TopIn", 200, 100, -100, 1, L"cm"},
    {L"Coilover top height", L"TopUp", 200, 100, -100, 1, L"cm"},
    {L"Coilover top forward", L"TopFwd", 200, 100, -100, 1, L"cm"},
    {L"Coilover size", L"Size", 50, 10, 0.5, 0.05, L"x"},
    {L"Lower arm pivot inboard", L"ArmIn", 200, 100, -100, 1, L"cm"},
    {L"Lower arm pivot height", L"ArmUp", 200, 100, -100, 1, L"cm"},
    {L"Lower arm mount spacing", L"ArmSpreadCm", 150, 50, 0, 1, L"cm"},  // 0 joins the two mounts into one (8 bits: the 5 words are full)
    {L"Shaft joint position", L"CvOff", 200, 20, -10, 0.5, L"cm"},
    {L"Spacer size", L"SpacerSize", 50, 10, 0.5, 0.05, L"x"}};  // diameter and bolt circle, to line the studs up with the wheel  // inboard of the hub boss; the stub follows
struct Fit { int v[MAXAXLE][NFIT]; };
struct Geo { double spSize = 1, cvOff = 0, spread = 1, brake = 0, spacer = 0, upIn = 0, upUp = 0, lowIn = 0, lowUp = 0, topIn = 0, topUp = 0, topFwd = 0, size = 1, armIn = 0, armUp = 0; };
static std::map<long long, Fit> fits;
static const int FIT_WORDS = 6;  // 24-bit words per axle (6 since the spacer size row: 5 held 120 bits)
static int bitsOf(int top) { int b = 0; while ((1 << b) <= top) b++; return b; }
static void packAxle(const int* v, uint32_t out[FIT_WORDS]) {
    uint64_t acc = 0; int have = 0, w = 0;
    for (int i = 0; i < FIT_WORDS; i++) out[i] = 0;
    for (int i = 1; i < NFIT; i++) {
        acc |= (uint64_t)v[i] << have; have += bitsOf(FIT[i].top);
        while (have >= 24 && w < FIT_WORDS) { out[w++] = (uint32_t)(acc & 0xFFFFFF); acc >>= 24; have -= 24; }
    }
    if (have > 0 && w < FIT_WORDS) out[w] = (uint32_t)(acc & 0xFFFFFF);
}
static void unpackAxle(const uint32_t in[FIT_WORDS], int* v) {
    uint64_t acc = 0; int have = 0, w = 0;
    for (int i = 1; i < NFIT; i++) {
        int b = bitsOf(FIT[i].top);
        while (have < b && w < FIT_WORDS) { acc |= (uint64_t)(in[w++] & 0xFFFFFF) << have; have += 24; }
        int x = (int)(acc & ((1u << b) - 1)); acc >>= b; have -= b;
        v[i] = x > FIT[i].top ? FIT[i].def : x;
    }
}
static int fitWord[MAXAXLE][FIT_WORDS];  // the entries' name indices
static int fitWordName(int a, int w) {
    if (!fitWord[a][w] && fToName && stringCDO) {
        wchar_t t[16]; swprintf_s(t, L"QXS%d%c", a + 1, L'A' + w);
        unsigned char n[0x40] = {0};
        FStr f = str(t);
        memcpy(n + off(fToName, "InString"), &f, sizeof f);
        api->process_event(stringCDO, fToName, n);
        fitWord[a][w] = get<int>((uintptr_t)n + off(fToName, "ReturnValue"));
    }
    return fitWord[a][w];
}
static bool isFitWord(uint32_t nm, int& a, int& w) {
    for (a = 0; a < MAXAXLE; a++) for (w = 0; w < FIT_WORDS; w++) if (nm && fitWordName(a, w) == (int)nm) return true;
    return false;
}
// What the car carries; false when it carries nothing of ours (a car never tuned keeps the defaults).
static bool readCarFit(uintptr_t v, Fit& out) {
    if (oBodyColors < 0) return false;
    uintptr_t d = get<uintptr_t>(v + oBodyColors);
    int n = get<int>(v + oBodyColors + 8);
    uint32_t words[MAXAXLE][FIT_WORDS] = {};
    int got[MAXAXLE] = {};
    for (int e = 0; d && e < n && e < 200; e++) {
        uintptr_t c = d + (uintptr_t)e * colSize;
        int a, w;
        if (get<uint32_t>(c + oColSlot + 4) || !isFitWord(get<uint32_t>(c + oColSlot), a, w)) continue;
        words[a][w] = get<uint32_t>(c + oColColor) & 0xFFFFFF;
        got[a] |= 1 << w;
    }
    bool any = false;
    for (int a = 0; a < MAXAXLE; a++) {
        for (int i = 1; i < NFIT; i++) out.v[a][i] = FIT[i].def;
        if (got[a] == (1 << FIT_WORDS) - 1) { unpackAxle(words[a], out.v[a]); any = true; }
    }
    return any;
}
static uintptr_t fSendFit, fGetPCFit;
static int pSendVeh = -1, pSendCus = -1, oCustom = -1, custSize = 0;
// The car's list goes back whole -- its own colours, the paint, plates, the tune -- with our words replaced.
static void sendFit(uintptr_t v, const Fit& k, int axles) {
    if (!fSendFit || !fGetPCFit || pSendVeh < 0 || pSendCus < 0 || oCustom < 0 || custSize <= 0 || custSize > 0x200 || colSize > 0x40) return;
    uintptr_t d = get<uintptr_t>(v + oBodyColors);
    int n = get<int>(v + oBodyColors + 8), have = 0;
    if (n < 0 || n > 200) return;
    static unsigned char list[0x40 * 260];
    for (int e = 0; d && e < n; e++) {
        uintptr_t c = d + (uintptr_t)e * colSize;
        int a, w;
        if (!get<uint32_t>(c + oColSlot + 4) && isFitWord(get<uint32_t>(c + oColSlot), a, w)) continue;
        rd(c, list + (uintptr_t)have * colSize, colSize);
        have++;
    }
    if (axles < 1) axles = 2;
    if (axles > MAXAXLE) axles = MAXAXLE;
    for (int a = 0; a < axles; a++) {
        uint32_t words[FIT_WORDS];
        packAxle(k.v[a], words);
        for (int w = 0; w < FIT_WORDS; w++) {
            unsigned char* e = list + (uintptr_t)have++ * colSize;
            memset(e, 0, colSize);
            uint32_t key[2] = {(uint32_t)fitWordName(a, w), 0}, col = words[w] | 0xFF000000u;
            memcpy(e + oColSlot, key, 8);
            memcpy(e + oColColor, &col, 4);
        }
    }
    struct { uintptr_t context; int index; uintptr_t ret; } pc = {v, 0, 0};
    api->process_event(gameplayCDO, fGetPCFit, &pc);
    if (!pc.ret) return;
    unsigned char q[0x240] = {0};
    memcpy(q + pSendVeh, &v, 8);
    rd(v + oCustom, q + pSendCus, custSize);
    struct { void* data; int num, max; } colors = {list, have, have};
    memcpy(q + pSendCus + (oBodyColors - oCustom), &colors, sizeof colors);
    api->process_event(pc.ret, fSendFit, q);
    MT_LOG("Brakes & Suspension: %d axles saved on the car (%d colours in its list)", axles, have);
}
static std::wstring fitSection(long long id) { return L"Vehicle" + std::to_wstring(id); }
static bool tunedHere(long long id) {  // this machine set it up: the ini has its section
    static std::map<long long, bool> known;
    auto k = known.find(id);
    if (k != known.end()) return k->second;
    wchar_t buf[8];
    bool has = id > 0 && GetPrivateProfileStringW(fitSection(id).c_str(), L"Axle1.Brake", L"", buf, 8, iniPath(L"suspension_tuning.ini").c_str()) > 0;
    return known[id] = has;
}
static Fit& fitOf(long long id) {
    auto f = fits.find(id);
    if (f != fits.end()) return f->second;
    Fit k{};
    bool ini = tunedHere(id);
    for (int a = 0; a < MAXAXLE; a++)
        for (int i = 1; i < NFIT; i++) {
            int v = ini ? iniInt(L"suspension_tuning.ini", fitSection(id).c_str(), (L"Axle" + std::to_wstring(a + 1) + L"." + FIT[i].key).c_str(), FIT[i].def) : FIT[i].def;
            k.v[a][i] = v < 0 || v > FIT[i].top ? FIT[i].def : v;
        }
    return fits[id] = k;
}
static void saveFitIni(long long id) {
    if (id <= 0) return;
    const Fit& k = fitOf(id);
    for (int a = 0; a < MAXAXLE; a++)
        for (int i = 1; i < NFIT; i++)
            iniSet(L"suspension_tuning.ini", fitSection(id).c_str(), (L"Axle" + std::to_wstring(a + 1) + L"." + FIT[i].key).c_str(), std::to_wstring(k.v[a][i]).c_str(), true);
}
static double fitNum(const Fit& k, int a, int i) { return FIT[i].zero + FIT[i].step * k.v[a][i]; }
static Geo geoOf(long long id, int axle) {
    const Fit& k = fitOf(id);
    int a = axle < 0 ? 0 : axle >= MAXAXLE ? MAXAXLE - 1 : axle;
    Geo g;
    g.brake = fitNum(k, a, F_BRAKE); g.spacer = fitNum(k, a, F_SPACER);
    g.upIn = fitNum(k, a, F_UPIN); g.upUp = fitNum(k, a, F_UPUP); g.lowIn = fitNum(k, a, F_LOWIN); g.lowUp = fitNum(k, a, F_LOWUP);
    g.topIn = fitNum(k, a, F_TOPIN); g.topUp = fitNum(k, a, F_TOPUP); g.topFwd = fitNum(k, a, F_TOPFWD);
    g.size = fitNum(k, a, F_SIZE); g.armIn = fitNum(k, a, F_ARMIN); g.armUp = fitNum(k, a, F_ARMUP);
    g.spread = fitNum(k, a, F_ARMSPREAD); g.cvOff = fitNum(k, a, F_CVOFF); g.spSize = fitNum(k, a, F_SPSIZE);
    return g;
}

// The fitted kit's row name, in any slot: "C0001_BBK360" (a Brake Pads part since 2026-11-09) gives a 360 mm disc;
// the old BrakePower rows "C0001_BBK360_Red" also carry a caliper colour. No kit: the BrakePower row, for the log.
static std::string fittedBrake(uintptr_t v) {
    if (oParts < 0) return "";
    uintptr_t data = get<uintptr_t>(v + oParts);
    int n = get<int>(v + oParts + 8);
    char buf[128];
    std::string power, oldKit;
    for (int i = 0; data && i < n && i < 256; i++) {
        uintptr_t part = data + (uintptr_t)i * partSize;
        if (!api->fname(part + oPartKey, buf, sizeof buf)) continue;
        if (const char* b = strstr(buf, "C0001_BBK")) {
            if (!strchr(b + 9, '_')) return buf;  // C0001_BBK400: the Brake Pads kit
            oldKit = buf;                         // C0001_BBK400_Gold: a hidden BrakePower kit, used only without one
        }
        if (get<uint8_t>(part + oPartSlot) == SLOT_BRAKE_POWER) power = buf;
    }
    return oldKit.empty() ? power : oldKit;
}
static void readKit(Car& c, const std::string& key) {
    c.kit = key; c.kitMm = 0; c.kitMat = M_GREY;
    size_t at = key.find("BBK");
    if (at == std::string::npos) return;
    c.kitMm = atoi(key.c_str() + at + 3);
    size_t u = key.find('_', at);
    for (int i = 0; u != std::string::npos && i < 6; i++)
        if (key.compare(u + 1, std::string::npos, KIT_COLOURS[i]) == 0) c.kitMat = M_RED + i;
}
// The colour the Paint screen stored for a slot (FColor BGRA packed), or 0.
static void readPaint(uintptr_t v, uint32_t out[NPAINT]) {
    for (int i = 0; i < NPAINT; i++) out[i] = 0;
    if (oBodyColors < 0) return;
    for (int i = 0; i < NPAINT; i++)
        paintIndex(i);
    uintptr_t data = get<uintptr_t>(v + oBodyColors);
    int n = get<int>(v + oBodyColors + 8);
    for (int e = 0; data && e < n && e < 128; e++) {
        uintptr_t c = data + (uintptr_t)e * colSize;
        uint32_t nm = get<uint32_t>(c + oColSlot);
        for (int i = 0; i < NPAINT; i++)
            if (paintName[i] && nm == (uint32_t)paintName[i] && !get<uint32_t>(c + oColSlot + 4)) out[i] = get<uint32_t>(c + oColColor) | 0x01000000;
    }
}

// ---- the four entries on the game's Paint screen ----
// The Paint screen lists MTVehicle.ColorSlots (TMap<FName, FMTVehicleColorSlot>) and ignores the ColorSlots of a
// brake part, which has no mesh (tested 2026-11-09: a kit fitted, still only Body_0/Body_1). So every decorated car
// gets the four QX_* entries added to that map in memory; the game paints them into Customization.BodyColors like
// any slot, and readPaint() reads them back. The game rebuilds the map when parts change, so a missing entry is added
// again on the next second. UE 5.5 TMap layout: TSparseArray (data, num, max | TBitArray inline[4], heap, numBits,
// maxBits | firstFree, numFree), then the hash (inline[1], heap, size); an element is the pair, HashNextId, HashIndex.
static int oColorSlots = -1, pairSize, oSlotText, oSlotText2, oSlotColor, oSlotMetal, oSlotRough, oT2Pattern, oT2Override;
static int pPadIn, pPadCount, pPadRet, pTextIn, pTextRet;
static uintptr_t fRightPad, fToText, textCDO;
static bool slotsBroken;
static const wchar_t* const PAINT_LABELS[NPAINT] = {L"Brake Caliper", L"Brake Disc", L"Suspension", L"Dampers", L"Wheel Spacer", L"Lug Nuts", L"Knuckle & Arms", L"Drive Shaft"};
// FColor BGRA: red, steel, grey, yellow, dark grey, silver
static const uint32_t PAINT_START[NPAINT] = {0xFF8C0503, 0xFF6E6E6E, 0xFF8C8C8C, 0xFFD99905, 0xFF3C3C3C, 0xFFB4B4B4, 0xFF505050, 0xFF1E1E1E};
template <class T> static void poke(uintptr_t p, T v) { api->write(p, &v, sizeof v); }

// A block the game's allocator owns, so the game may free it later: the buffer of a padded FString it builds.
static uintptr_t gameAlloc(size_t bytes) {
    unsigned char b[0x40] = {0};
    FStr none = str(L"");
    memcpy(b + pPadIn, &none, sizeof none);
    int chars = (int)(bytes / 2) + 8;
    memcpy(b + pPadCount, &chars, 4);
    api->process_event(stringCDO, fRightPad, b);
    uintptr_t data = get<uintptr_t>((uintptr_t)b + pPadRet);
    int max = get<int>((uintptr_t)b + pPadRet + 12);
    return data && (size_t)max * 2 >= bytes ? data : 0;  // the FString is dropped, not freed: the block is the map's now
}

// An FText the game made, its one reference handed to whoever stores it.
static void makeText(const wchar_t* s, unsigned char out[16]) {
    unsigned char b[0x40] = {0};
    FStr in = str(s);
    memcpy(b + pTextIn, &in, sizeof in);
    api->process_event(textCDO, fToText, b);
    memcpy(out, b + pTextRet, 16);
}

static void addPaintSlots(uintptr_t v) {
    if (oColorSlots < 0 || slotsBroken) return;
    for (int i = 0; i < NPAINT; i++)
        paintIndex(i);
    uintptr_t m = v + oColorSlots;
    const int stride = pairSize + 8;
    uintptr_t data = get<uintptr_t>(m);
    int num = get<int>(m + 8), max = get<int>(m + 12);
    uintptr_t bitsHeap = get<uintptr_t>(m + 32);
    int numBits = get<int>(m + 40), maxBits = get<int>(m + 44), firstFree = get<int>(m + 48), numFree = get<int>(m + 52);
    int hashSize = get<int>(m + 72);
    uintptr_t hash = hashSize > 1 ? get<uintptr_t>(m + 64) : m + 56, bits = bitsHeap ? bitsHeap : m + 16;
    if (!data || hashSize <= 0 || (hashSize & (hashSize - 1)) || numBits != num || num > max || !hash) return;  // empty or unfamiliar: leave it
    auto used = [&](int i) { return (get<uint32_t>(bits + (uintptr_t)(i >> 5) * 4) >> (i & 31)) & 1; };
    bool have[NPAINT] = {};
    for (int i = 0; i < num; i++) {
        if (!used(i)) continue;
        uintptr_t e = data + (uintptr_t)i * stride;
        uint32_t cmp = get<uint32_t>(e), nb = get<uint32_t>(e + 4);
        int hi = get<int>(e + pairSize + 4), hn = get<int>(e + pairSize);
        if (hi < 0 || hi >= hashSize || hn < -1 || hn >= num) {
            slotsBroken = true;
            MT_LOG("paint slots: the car's colour-slot map is not laid out as expected -- no Paint entries");
            return;
        }
        for (int k = 0; k < NPAINT; k++) have[k] |= paintName[k] && cmp == (uint32_t)paintName[k] && !nb;
    }
    int added = 0;
    for (int k = 0; k < NPAINT; k++) {
        if (have[k] || !paintName[k]) continue;
        int idx = numFree > 0 ? firstFree : num;
        if (idx < 0 || idx >= maxBits) { if (!added) MT_LOG("paint slots: no room in the map's flags -- no Paint entries on this car"); break; }
        if (numFree > 0) {  // a hole: unlink it from the free list (FSparseArrayFreeListLink {prev, next})
            int next = get<int>(data + (uintptr_t)idx * stride + 4);
            if (next >= 0) poke<int>(data + (uintptr_t)next * stride, -1);
            firstFree = next; numFree--;
            poke<int>(m + 48, firstFree); poke<int>(m + 52, numFree);
        } else {
            if (num == max) {  // grow by 8 into a block the game can free
                uintptr_t block = gameAlloc((size_t)(max + 8) * stride);
                if (!block) { MT_LOG("paint slots: no memory for the map -- no Paint entries"); slotsBroken = true; return; }
                std::vector<unsigned char> old((size_t)num * stride);
                api->read(data, old.data(), old.size());
                api->write(block, old.data(), old.size());
                // shortcut: the old block is left allocated (a few hundred bytes per car per growth); freeing it needs
                // FMemory::Free, which the loader does not expose. Upgrade if the loader ever gets an allocator.
                data = block; max += 8;
                poke<uintptr_t>(m, data); poke<int>(m + 12, max);
            }
            num++;
            poke<int>(m + 8, num);
            poke<int>(m + 40, num);
        }
        uintptr_t flag = bits + (uintptr_t)(idx >> 5) * 4;
        poke<uint32_t>(flag, get<uint32_t>(flag) | (1u << (idx & 31)));
        std::vector<unsigned char> e(stride, 0);
        uint32_t key[2] = {(uint32_t)paintName[k], 0};
        memcpy(e.data(), key, 8);
        unsigned char* val = e.data() + 8;  // FMTVehicleColorSlot: every FText a real one, the arrays empty
        makeText(PAINT_LABELS[k], val + oSlotText);
        makeText(L"", val + oSlotText2 + oT2Pattern);
        makeText(PAINT_LABELS[k], val + oSlotText2 + oT2Override);
        memcpy(val + oSlotColor, &PAINT_START[k], 4);
        float metal = 0.0f, rough = 0.5f;
        memcpy(val + oSlotMetal, &metal, 4); memcpy(val + oSlotRough, &rough, 4);
        api->write(data + (uintptr_t)idx * stride, e.data(), e.size());  // chained below
        added++;
    }
    if (added) {
        // The game's FName hash is not index + number on this build (seven entries disagreed), so instead of
        // bucketing by it, every entry goes into one bucket: HashSize 1, one chain. Lookups walk the chain and find
        // anything; the game's next Add to this map rehashes it with its own hash. With a heap hash buffer the
        // allocator keeps using it (TInlineAllocator prefers the secondary allocation), so bucket 0 is hash[0].
        int head = -1;
        for (int i = 0; i < num; i++) {
            if (!used(i)) continue;
            uintptr_t e = data + (uintptr_t)i * stride;
            poke<int>(e + pairSize, head);
            poke<int>(e + pairSize + 4, 0);
            head = i;
        }
        poke<int>(hash, head);
        poke<int>(m + 72, 1);
        MT_LOG("%s: %d paint slots added to the Paint screen", objName(classOf(v)).c_str(), added);
    }
}

// The Paint screen edits only colours the car already holds in Customization.BodyColors (tested 2026-11-09: our
// entries were listed but no pick ever reached the car, and Confirm sent nothing), so the four go in there too, at
// their start colours. FMTVehicleCustomizationColor is plain data: name, FColor, metallic, roughness.
static std::atomic<uintptr_t> gWatch;  // the nearest decorated car, for the paint probe
static std::unordered_map<uintptr_t, std::array<uint32_t, NPAINT>> lastQX;  // the colours a car's entries held last
static bool paintFromIni(long long id, std::array<uint32_t, NPAINT>& out) {
    if (id <= 0) return false;
    bool any = false;
    for (int j = 0; j < NPAINT; j++) {
        std::wstring v = iniGet(L"suspension_tuning.ini", (L"Vehicle" + std::to_wstring(id)).c_str(), (L"Paint." + std::wstring(PAINT_WIDE[j])).c_str());
        if (v.empty()) continue;
        out[j] = (uint32_t)wcstoul(v.c_str(), nullptr, 16); any = true;
    }
    return any;
}
static void addBodyColors(uintptr_t v) {
    if (oBodyColors < 0 || slotsBroken || colSize < 20) return;
    nameMeshSlots();
    uintptr_t a = v + oBodyColors;
    uintptr_t data = get<uintptr_t>(a);
    int num = get<int>(a + 8), max = get<int>(a + 12);
    if (num < 0 || num > max || max > 4096 || (max && !data)) return;
    bool have[NPAINT] = {};
    for (int e = 0; e < num; e++) {
        uintptr_t c = data + (uintptr_t)e * colSize;
        for (int k = 0; k < NPAINT; k++) have[k] |= paintName[k] && get<uint32_t>(c + oColSlot) == (uint32_t)paintName[k] && !get<uint32_t>(c + oColSlot + 4);
    }
    auto seen = lastQX.find(v);
    long long vid = oId >= 0 ? get<long long>(v + oId) : 0;
    if (seen == lastQX.end()) {  // first look at this car: what this machine saved for it, else the start colours
        std::array<uint32_t, NPAINT> st;
        for (int j = 0; j < NPAINT; j++) st[j] = PAINT_START[j];
        paintFromIni(vid, st);
        lastQX[v] = st; seen = lastQX.find(v);
    }
    std::array<uint32_t, NPAINT> before = seen->second;
    for (int e = 0; e < num; e++) {  // remember what the player painted, so a re-add puts it back rather than the start colour
        uintptr_t c = data + (uintptr_t)e * colSize;
        for (int k = 0; k < NPAINT; k++)
            if (paintName[k] && get<uint32_t>(c + oColSlot) == (uint32_t)paintName[k] && !get<uint32_t>(c + oColSlot + 4)) {
                seen->second[k] = get<uint32_t>(c + oColColor);
            }
    }
    if (vid > 0 && seen->second != before)  // painted: kept for the next load
        for (int j = 0; j < NPAINT; j++)
            if (seen->second[j] != before[j]) {
                wchar_t h[16]; swprintf_s(h, L"%08X", seen->second[j]);
                iniSet(L"suspension_tuning.ini", (L"Vehicle" + std::to_wstring(vid)).c_str(), (L"Paint." + std::wstring(PAINT_WIDE[j])).c_str(), h, true);
            }
    int need = 0;
    for (int k = 0; k < NPAINT; k++) need += paintName[k] && !have[k];
    if (!need) return;
    if (num + need > max) {
        int nmax = num + need + 4;
        uintptr_t block = gameAlloc((size_t)nmax * colSize);
        if (!block) return;
        if (num) { std::vector<unsigned char> old((size_t)num * colSize); api->read(data, old.data(), old.size()); api->write(block, old.data(), old.size()); }
        // shortcut: the old block leaks (num * 20 bytes, once per car), as in addPaintSlots.
        data = block; max = nmax;
        poke<uintptr_t>(a, data); poke<int>(a + 12, max);
    }
    for (int k = 0; k < NPAINT; k++) {
        if (!paintName[k] || have[k]) continue;
        std::vector<unsigned char> e(colSize, 0);
        uint32_t key[2] = {(uint32_t)paintName[k], 0};
        float metal = 0.0f, rough = 0.5f;
        memcpy(e.data() + oColSlot, key, 8);
        uint32_t col = seen->second[k];
        memcpy(e.data() + oColColor, &col, 4);
        memcpy(e.data() + oColColor + 4, &metal, 4); memcpy(e.data() + oColColor + 8, &rough, 4);
        api->write(data + (uintptr_t)num * colSize, e.data(), e.size());
        num++;
    }
    poke<int>(a + 8, num);
    MT_LOG("%s: %d brake/suspension colours added to the car's paint", objName(classOf(v)).c_str(), need);
}
static std::vector<Car> cars;

// The wheel's axle in its parent's space: FRotationMatrix's Y axis, which the spin (pitch) leaves alone.
static V axleOf(uintptr_t wheel) {
    double r[3]; rd(wheel + oRelRot, r, sizeof r);  // Pitch, Yaw, Roll in degrees
    double sp = sin(r[0] * PI / 180), cp = cos(r[0] * PI / 180), sy = sin(r[1] * PI / 180), cy = cos(r[1] * PI / 180),
           sr = sin(r[2] * PI / 180), cr = cos(r[2] * PI / 180);
    return {sr * sp * cy - cr * sy, sr * sp * sy + cr * cy, -sr * cp};
}
static double steerOf(uintptr_t wheel) {
    V y = axleOf(wheel);
    double a = atan2(-y.x, y.y);
    return a - PI * floor(a / PI + 0.5);  // a left wheel's mesh is turned half round
}

static void measure(Corner& k, V scale) {
    uintptr_t mesh = get<uintptr_t>(k.wheel.ptr + oStaticMesh);
    Box b = mesh ? boundsOf(mesh) : Box{{0, 0, 0}, {66, 22, 66}};
    k.R = b.size.z * fabs(scale.z) / 2;
    k.W = b.size.y * fabs(scale.y);
    if (k.R < 5) k.R = 33;
    if (k.W < 3) k.W = 20;
    // Where the tyre's middle and inner face sit along "inboard", off the wheel's pivot.
    double s = (axleOf(k.wheel.ptr).y >= 0 ? 1 : -1) * -k.side;  // the mesh's +Y, seen inboard
    double lo = (b.centre.y - b.size.y / 2) * scale.y * s, hi = (b.centre.y + b.size.y / 2) * scale.y * s;
    k.cIn = (lo + hi) / 2;
    k.eIn = lo > hi ? lo : hi;
}

static void pose(Corner& k, V hub, double steer, int kitMm, const Geo& g) {
    Q qs = axisAngle({0, 0, 1}, steer);
    V in = rot(qs, {0, -k.side, 0}), up{0, 0, 1};
    double R = k.R;
    // brake: disc inside the rim, caliper at the top rear of it
    // a kit's disc as a share of the tyre radius, so it fills the rim on any car: 320 / 360 / 400 mm
    double rd = kitMm >= 400 ? R * 0.64 : kitMm >= 360 ? R * 0.59 : kitMm ? R * 0.54 : R * 0.45;
    double thick = R * 0.085 + 0.5;
    // A wheel spacer (the game's Suspension part) pushes the wheel out of its blueprint spot: the brake and knuckle stay
    // on the hub, the spacer below grows to the wheel.
    // shortcut: the game's Wheel Spacers part is not read -- hub vs blueprint gave 15/30 cm on a car with none
    // (the track is wider than the blueprint's). The Tuning row "Brake position" closes any gap by hand.
    k.spacer = k.side * (hub.y - k.rest.y);
    double sp = 0;
    double bIn = kInset * k.W + sp - g.brake;  // the disc's distance inboard of the tyre's middle
    V at = add(hub, mul(in, k.cIn + bIn));
    if (brakeMesh) {
        // the unit's frame: axle along Y, origin mid-disc, disc radius 30; its caliper's fat side is -Y,
        // inboard on a right wheel, so a left wheel's set is mirrored in Y
        double sc = rd / 30;
        V s{sc, sc * k.side, sc};
        Q qw = qmul(qs, axisAngle(V{0, 1, 0}, k.spin));  // turning with the wheel
        place(k.piece[P_DISC], at, qw, s);
        place(k.piece[P_CALIPER], at, qs, s);
        // spacer: from the hat's face (unit y +6, toward the wheel) out to the tyre's middle, where the rim bolts on
        double gap = kInset * k.W + sp - 6 * sc + g.spacer;  // its own length: moving the brake no longer shrinks it
        if (gap < 0.5) gap = 0.5;
        V face = sub(at, mul(in, 6 * sc));
        double sd = sc * g.spSize;  // the Spacer size row: diameter and bolt circle, never the length
        place(k.piece[P_SPACER], face, qw, V{sd, k.side * gap / 10, sd});
        place(k.piece[P_SPLUGS], sub(face, mul(in, gap)), qw, V{sd, k.side * sc, sd});  // on the outer face, unscaled in length
    } else {
    V disc = at;
    put(k.piece[P_DISC], cylBox, disc, qmul(qs, axisAngle({1, 0, 0}, PI / 2)), {rd * 2, rd * 2, thick});
    double th = 35 * PI / 180, rc = rd * 0.88;
    put(k.piece[P_CALIPER], cubeBox, add(disc, rot(qs, {-sin(th) * rc, 0, cos(th) * rc})), qmul(qs, axisAngle({0, 1, 0}, -th)),
        {rd * 0.6, thick + R * 0.12, rd * 0.32});
    }
    // coilover: grown from the socket (the blueprint wheel position, on the body), inboard of the
    // tyre and up inside the arch, to the knuckle just inboard of the tyre's inner face
    V low = add(add(hub, mul(in, k.eIn + R * 0.06)), mul(up, R * 0.05));
    V top = add(k.rest, V{g.topFwd, -k.side * (k.eIn + R * 0.35 + g.topIn), R * 0.72 + g.topUp});
    V d = sub(top, low);
    double L = len(d);
    if (L < R * 0.3) { low = sub(top, mul(L > 1 ? mul(d, 1 / L) : up, R * 0.3)); d = sub(top, low); L = R * 0.3; }  // bottomed out
    V u = mul(d, 1 / L);
    double D = (R * 0.32 < 6 ? 6 : R * 0.32 > 60 ? 60 : R * 0.32) * g.size;  // a kart's to a monster truck's (wheel size tuning)
    if (suspMesh) {
    } else if (k.prop) {
        V sz = shockBox.size, size = sz.x >= sz.y && sz.x >= sz.z ? V{L, D, D} : sz.y >= sz.z ? V{D, L, D} : V{D, D, L};
        put(k.piece[P_SHOCK], shockBox, add(low, mul(d, 0.5)), qmul(fromZ(u), shockPre), size);
    } else {
        put(k.piece[P_SHOCK], cylBox, add(low, mul(d, 0.5)), fromZ(u), {D * 0.5, D * 0.5, L});
    }
    if (suspMesh) {
        // Arini's set. The knuckle is in the brake unit's frame, its +Y inboard of the disc; its mounts
        // (unit cm): coilover eye (0,9,28), ball joint (0,9,-23), outer CV (0,6,0).
        double sc = rd / 30;
        // The knuckle reaches inboard until its two mounts (coilover eye, ball joint) clear the tyre's inner face by
        // half a coilover and a bit, so neither the coilover nor the arm cuts through the rim (2026-11-09).
        double reach = k.eIn + D / g.size * 0.55 + 1.5 - (k.cIn + bIn);  // the size grows the coilover, not the knuckle
        if (reach < 9 * sc) reach = 9 * sc;
        double upR = reach + g.upIn, lowR = reach + g.lowIn;  // each mount its own way in (the user: the top and the bottom
        if (upR < 4.5 * sc) upR = 4.5 * sc;                     // both touched the wheel on some cars)
        if (lowR < 4.5 * sc) lowR = 4.5 * sc;
        V U, Jm, C;
        if (usable(k.piece[P_WEB])) {  // the split set: hub fixed, web stretched, mounts moved -- end detail never squashed
            V sk{sc, -sc * k.side, sc};
            // the mounts as if the wheel pointed straight ahead: they are the steering axis, the knuckle turns about them
            if (!k.haveStraight || fabs(steer) < 0.02) { k.haveStraight = true; k.straightX = hub.x; k.straightY = hub.y; }
            V in0{0, -k.side, 0}, at0 = add(V{k.straightX, k.straightY, hub.z}, mul(in0, k.cIn + bIn));
            Q q0 = axisAngle(V{0, 0, 1}, 0);
            V Mu = add(at0, V{0, -k.side * upR, g.upUp}), Ml = add(at0, V{0, -k.side * lowR, g.lowUp});
            place(k.piece[P_KNUCKLE], at, qs, sk);
            place(k.piece[P_MOUNTS], Mu, q0, sk);   // the eye (or both mounts, without the split meshes)
            place(k.piece[P_MOUNTS2], Ml, q0, sk);  // the ball joint
            U = add(Mu, V{0, 0, 28 * sc}); Jm = add(Ml, V{0, 0, -23 * sc}); C = add(at, mul(in, 6 * sc));
            if (usable(k.piece[P_WEB2])) {  // two rods: hub bosses (0,4.5,+-12) to the eye and the ball joint
                V h = add(at, mul(in, 4.5 * sc));
                V rods[2][2] = {{add(h, rot(qs, V{0, 0, 12 * sc})), U}, {add(h, rot(qs, V{0, 0, -12 * sc})), Jm}};
                for (int r = 0; r < 2; r++) {
                    V d = sub(rods[r][1], rods[r][0]); double l = len(d);
                    if (l > 0.5) place(k.piece[r ? P_WEB2 : P_WEB], rods[r][0], yAlong(mul(d, 1 / l)), V{sc, l / 10, sc});
                }
            } else place(k.piece[P_WEB], add(at, mul(in, 4.5 * sc)), qs, V{sc, -k.side * (reach - 4.5 * sc) / 4.5, sc});
        } else {
            V sk{sc, -k.side * reach / 9, sc};
            auto mount = [&](double x, double y, double z) { return add(at, rot(qs, V{x * sk.x, y * sk.y, z * sk.z})); };
            place(k.piece[P_KNUCKLE], at, qs, sk);
            U = mount(0, 9, 28); Jm = mount(0, 9, -23); C = mount(0, 6, 0);
        }
        // coilover: bottom eye on the knuckle, top eye at the body mount; 62.76 cm eye to eye, ~11.8 across
        V cd = sub(top, U); double cl = len(cd);
        if (cl < R * 0.3) cl = R * 0.3;
        V cu = len(cd) > 1 ? mul(cd, 1 / len(cd)) : up;
        double r = D / 11.8;  // the coilover's thickness scale
        if (usable(k.piece[P_CBOT])) {
            // Arini's split (coil_split.py): the eyes and perches stay rigid; only the spring and rod between them
            // stretch -- body from bottom eye +7.88 to top eye -8.02, unit 0..50 (the user: ends never squash)
            Q qc = along(cu, V{0, -k.side, 0});  // not with the steering: the coilover stays put
            V T = add(U, mul(cu, cl));
            place(k.piece[P_CBOT], U, qc, V{r, r, r});
            place(k.piece[P_CTOP], T, qc, V{r, r, r});
            double gap = cl - (7.88 + 8.02) * r;  // SM_CoiloverTop re-cut compact (perch 8 cm under the eye)
            if (gap < 1) gap = 1;
            place(k.piece[P_SHOCK], add(U, mul(cu, 7.88 * r)), qc, V{r, r, gap / 50});
            if (usable(k.piece[P_DBODY])) {
                // Arini's telescoping damper: the cylinder rides on the bottom perch, the rod hangs from the top one and
                // slides into it. Both rigid; their length scale is set once from this corner's coilover (Arini's
                // pair suits a 46.86 gap), so a kart's and a truck's both telescope instead of squeezing.
                if (k.damperZ <= 0) k.damperZ = gap / (46.86 * r) < 0.3 ? 0.3 : gap / (46.86 * r);
                double z = r * k.damperZ;
                place(k.piece[P_DBODY], add(U, mul(cu, 7.88 * r)), qc, V{r, r, z});
                place(k.piece[P_DROD], sub(T, mul(cu, 8.02 * r)), qc, V{r, r, z});
            }
        } else place(k.piece[P_SHOCK], U, along(cu, V{0, -k.side, 0}), V{r, r, cl / 62.76});
        // lower arm: ball joint at the origin, bushings at +Y 100 (50 apart), to the body pivot
        V P = add(k.rest, V{0, -k.side * (k.eIn + R * 1.1 + g.armIn), -R * 0.32 + g.armUp});
        V ad = sub(P, Jm); double al = len(ad), aw = R / 40 < 0.4 ? 0.4 : R / 40 > 4 ? 4 : R / 40;
        if (usable(k.piece[P_LINK1])) {
            // Arini's split: the ball end and the two chassis bushings are rigid, only the links stretch. The
            // bushings sit fore and aft of the pivot (bore along the car), their spacing a Tuning row; 0 joins them.
            place(k.piece[P_ARM], Jm, al > 1e-3 ? yAlong(mul(ad, 1 / al)) : yAlong(V{0, -k.side, 0}), V{aw, aw, aw});
            double half = g.spread / 2;  // the row is the bushings' spacing in cm (the user: centimetres, not a multiplier)
            V Bs[2] = {add(P, V{half, 0, 0}), add(P, V{-half, 0, 0})};
            Q straight = axisAngle(V{0, 0, 1}, 0);
            for (int b = 0; b < 2; b++) {
                place(k.piece[b ? P_BUSH2 : P_BUSH1], Bs[b], straight, V{aw, aw, aw});
                V ld = sub(Bs[b], Jm); double ll = len(ld);
                V u = ll > 1e-3 ? mul(ld, 1 / ll) : V{0, -k.side, 0};
                // rigid, pivoting at the knuckle (the user, 2026-11-09): the link hangs from the ball at its fixed
                // length and the travel's small arc is lost in the bushing. Taken again if the gap outgrows the bushing
                // (a length taken mid-setup left both links hanging in the air)
                double r0 = ll - (3.0 + 2.5) * aw; if (r0 < 0.01) r0 = 0.01;
                if (k.linkRun[b] <= 0 || fabs(r0 - k.linkRun[b]) > 1.5 * aw) k.linkRun[b] = r0;
                place(k.piece[b ? P_LINK2 : P_LINK1], add(Jm, mul(u, 3.0 * aw)), yAlong(u), V{aw, k.linkRun[b] / 10, aw});
            }
        } else if (al > 1) place(k.piece[P_ARM], Jm, yAlong(mul(ad, 1 / al)), V{aw, al / 100, aw});
        if (k.driven) {
            V S = add(k.rest, V{0, -k.side * (k.eIn + R * 1.3), 0});
            if (S.y * k.side < 8) S.y = k.side * 8;
            V C0 = add(at, mul(in, 4.5 * sc));  // inside the knuckle's hub boss, which wraps it (through the brake read wrong)
            (void)C;
            double sw = R / 33 < 0.5 ? 0.5 : R / 33 > 4 ? 4 : R / 33;
            // straight out of the hub along the wheel's own axle, so it lines up with the wheel; its end is the boot
            // (the shaft mesh's CV end), and from there the shaft bends off toward the car (the user, 2026-11-09)
            double stub = 3.5 * sc + g.cvOff;  // just past the hub boss's bearing cap (y 7.2), plus the Tuning row
            if (stub < 2) stub = 2;
            V B = add(C0, mul(in, stub));
            if (usable(k.piece[P_BOOTW])) {
                // Arini's modular joint (2026-11-09): a ball at each joint, a boot along each shaft out of it (rubber
                // from inside the ball, clamp band at 8.8..9.6), every tube starting 9.0 from a ball's centre. All one
                // uniform scale -- a boot scaled in X/Z only opens a gap at its clamp.
                double bs = sw;
                V axle = rot(qs, V{0, 1, 0});
                // a piece along `a` spinning with the wheel: the wheel turns k.spin about +axle, so about `a` by
                // k.spin times the side `a` points to (CV joints pass the same speed)
                auto spun = [&](V a) { return qmul(yAlong(a), axisAngle(V{0, 1, 0}, (dot(a, axle) >= 0 ? 1 : -1) * k.spin)); };
                V Jc = add(C0, mul(in, stub + 9 * bs));  // the outer ball: the stub ends inside its wheel-side clamp
                place(k.piece[P_SHAFT2], C0, spun(in), V{bs, stub / 10, bs});
                place(k.piece[P_BOOT], Jc, spun(in), V{bs, bs, bs});
                place(k.piece[P_BOOTW], Jc, spun(mul(in, -1)), V{bs, bs, bs});
                V sd = sub(S, Jc); double sl = len(sd);
                V d = sl > 1e-3 ? mul(sd, 1 / sl) : in;
                place(k.piece[P_BOOTC], Jc, spun(d), V{bs, bs, bs});
                place(k.piece[P_CVIN], S, spun(d), V{bs, bs, bs});
                place(k.piece[P_BOOTI], S, spun(mul(d, -1)), V{bs, bs, bs});
                // rigid tube from the outer joint; its inner end plunges in the inner boot as the wheel travels
                if (k.shaftRun <= 0 || fabs(sl - 18 * bs - k.shaftRun) > 4 * bs) k.shaftRun = sl - 18 * bs;  // inside the boot
                double run = k.shaftRun;
                if (run > 0.5) place(k.piece[P_SHAFT], add(Jc, mul(d, 9 * bs)), spun(d), V{bs, run / 10, bs});
                return;
            }
            bool parts = usable(k.piece[P_BOOT]);  // SM_ShaftStub (3 cm pipe, 0..10) + SM_CVBoot (10.5 long, cup 6.7..10.5)
            double bs = sw;                        // one thickness scale for pipe, boot and shaft: they stay flush
            if (usable(k.piece[P_SHAFT2])) place(k.piece[P_SHAFT2], C0, yAlong(in), V{bs, stub / (parts ? 10 : 100), bs});
            else B = C0;
            V sd = sub(S, B); double sl = len(sd);
            if (sl > 1) {
                V dir = mul(sd, 1 / sl);
                if (parts && usable(k.piece[P_CVIN])) {
                    // Arini's set: outer joint at the stub's end, inner one at the car, the plain tube between, each
                    // end sunk 9 into its joint's clamp band. Each joint faces halfway between its two shafts.
                    auto unit = [](V v) { double l = len(v); return l > 1e-6 ? mul(v, 1 / l) : V{0, 0, 1}; };
                    V jo = unit(add(in, dir)), ji = unit(add(V{0, -k.side, 0}, dir));
                    place(k.piece[P_BOOT], B, yAlong(jo), V{bs, bs, bs});
                    place(k.piece[P_CVIN], S, yAlong(ji), V{bs, bs, bs});
                    V a0 = add(B, mul(jo, 9 * bs)), a1 = sub(S, mul(ji, 9 * bs)), td = sub(a1, a0);
                    double run = len(td);
                    if (run > 0.5) place(k.piece[P_SHAFT], a0, yAlong(mul(td, 1 / run)), V{bs, run / 10, bs});
                } else {
                    if (parts) { place(k.piece[P_BOOT], B, yAlong(dir), V{bs, bs, bs}); B = add(B, mul(dir, 9 * bs)); sd = sub(S, B); sl = len(sd); }
                    if (sl > 1) place(k.piece[P_SHAFT], B, yAlong(mul(sd, 1 / sl)), V{sw, sl / 100, sw});
                }
            }
        }
        return;
    }
    // what ties it to the car: a knuckle behind the disc from the lower ball joint up to the coilover's
    // foot, a lower arm from that joint to a pivot fixed on the body, and on a driven wheel a half-shaft
    // from the hub to the body's middle. The body ends sit at the socket, so travel swings them.
    V kn = add(at, mul(in, R * 0.1)), J = add(kn, mul(up, -R * 0.38));
    V kd = sub(low, J); double kl = len(kd);
    if (kl > 1) put(k.piece[P_KNUCKLE], cubeBox, add(J, mul(kd, 0.5)), along(mul(kd, 1 / kl), in), {R * 0.22, R * 0.1, kl});
    V P = add(k.rest, V{0, -k.side * (k.eIn + R * 1.1), -R * 0.32});
    if (P.y * k.side < 10) P.y = k.side * 10;
    V ad = sub(P, J); double al = len(ad);
    if (al > 1) put(k.piece[P_ARM], cubeBox, add(J, mul(ad, 0.5)), along(mul(ad, 1 / al), up), {R * 0.28, 2.5, al});
    if (k.driven) {
        V S = add(k.rest, V{0, -k.side * (k.eIn + R * 1.3), 0});
        if (S.y * k.side < 8) S.y = k.side * 8;
        V sd = sub(S, kn); double sl = len(sd), sw = R * 0.13 < 3 ? 3 : R * 0.13 > 20 ? 20 : R * 0.13;
        if (sl > 1) put(k.piece[P_SHAFT], cylBox, add(kn, mul(sd, 0.5)), fromZ(mul(sd, 1 / sl)), {sw, sw, sl});
    }
}

static V readV(uintptr_t at) { V v{}; rd(at, &v, sizeof v); return v; }

static std::mutex lock;  // the worker's lists, swapped in by the tick with try_lock
static std::vector<Ref> found, candidates;
static std::map<uint64_t, V> foundRest, rests;  // (class index << 32 | wheel name) -> the CDO wheel's RelativeLocation

static void decorate(const Ref& veh) {
    Car car{veh, {}};
    uintptr_t arr = get<uintptr_t>(veh.ptr + oWheels);
    int n = get<int>(veh.ptr + oWheels + 8);
    if (n < 3 || n > 32) { cars.push_back(car); return; }  // a bike, or nothing yet: kept so it is not tried every frame
    uintptr_t clsIndex = get<int>(classOf(veh.ptr) + L().obj_index);
    std::vector<uintptr_t> wheels;
    for (int i = 0; i < n; i++) {
        uintptr_t w = get<uintptr_t>(arr + 8ull * i);
        if (w && usable(refOf(w))) wheels.push_back(w);
    }
    for (uintptr_t w : wheels) {
        V at = readV(w + oRelLoc);
        bool inner = true;  // of a dual pair, only the inner wheel gets a set
        for (uintptr_t o : wheels)
            if (o != w) {
                V b = readV(o + oRelLoc);
                if (fabs(b.x - at.x) < 15 && b.y * at.y > 0 && fabs(b.y) < fabs(at.y) - 5) inner = false;
            }
        if (!inner) continue;
        Corner k{};
        k.wheel = refOf(w);
        uint64_t key = clsIndex << 32 | get<uint32_t>(w + L().obj_name);
        auto r = rests.find(key);
        for (uintptr_t c = superOf(classOf(veh.ptr)); r == rests.end() && c; c = superOf(c))  // a child blueprint inherits its parent's wheels
            r = rests.find((uint64_t)get<int>(c + L().obj_index) << 32 | get<uint32_t>(w + L().obj_name));
        k.rest = r != rests.end() ? r->second : at;
        if (r == rests.end()) MT_LOG("  %s: no blueprint position, resting on the first one seen", objName(w).c_str());
        k.side = at.y >= 0 ? 1 : -1;
        k.lastScale = readV(w + oRelScale);
        measure(k, k.lastScale);
        uintptr_t parent = get<uintptr_t>(w + oParent);
        k.prop = usable(shock);
        k.piece[P_DISC] = makePiece(veh.ptr, parent, brakeMesh ? disc.ptr : cylinder.ptr, M_DISC);
        k.piece[P_CALIPER] = makePiece(veh.ptr, parent, brakeMesh ? caliper.ptr : cube.ptr, M_GREY);
        bool coSplit = suspMesh && coBot.ptr && coTop.ptr && coBody.ptr;
        if (coSplit) { k.piece[P_CBOT] = makePiece(veh.ptr, parent, coBot.ptr, M_YELLOW); k.piece[P_CTOP] = makePiece(veh.ptr, parent, coTop.ptr, M_YELLOW); }
        bool tele = coSplit && coSpring.ptr && dBody.ptr && dRod.ptr;  // spring + rigid cylinder + rigid rod
        if (tele) { k.piece[P_DBODY] = makePiece(veh.ptr, parent, dBody.ptr, M_YELLOW); k.piece[P_DROD] = makePiece(veh.ptr, parent, dRod.ptr, M_YELLOW); }
        k.piece[P_SHOCK] = suspMesh ? makePiece(veh.ptr, parent, tele ? coSpring.ptr : coSplit ? coBody.ptr : coil.ptr, M_GREY)
                                    : makePiece(veh.ptr, parent, k.prop ? shock.ptr : cylinder.ptr, k.prop ? -1 : M_BODY);
        if (suspMesh) setMat(k.piece[P_SHOCK], mats[M_YELLOW], 1);
        k.driven = (oDiffComp >= 0 && get<uintptr_t>(w + oDiffComp)) || (oShaftComp >= 0 && get<uintptr_t>(w + oShaftComp));
        bool split = suspMesh && kHub.ptr && kWeb.ptr && kMounts.ptr;
        k.piece[P_KNUCKLE] = makePiece(veh.ptr, parent, split ? kHub.ptr : suspMesh ? knuckle.ptr : cube.ptr, M_GREY);
        if (split) {
            bool two = kEye.ptr && kBall.ptr;
            k.piece[P_MOUNTS] = makePiece(veh.ptr, parent, two ? kEye.ptr : kMounts.ptr, M_GREY);
            if (two) k.piece[P_MOUNTS2] = makePiece(veh.ptr, parent, kBall.ptr, M_GREY);
            k.piece[P_WEB] = makePiece(veh.ptr, parent, kArm.ptr ? kArm.ptr : kWeb.ptr, M_GREY);
            if (kArm.ptr) k.piece[P_WEB2] = makePiece(veh.ptr, parent, kArm.ptr, M_GREY);
        }
        if (brakeMesh && spBody.ptr && spLugs.ptr) {  // the studs ride on the end, never stretched
            k.piece[P_SPACER] = makePiece(veh.ptr, parent, spBody.ptr, M_GREY);
            k.piece[P_SPLUGS] = makePiece(veh.ptr, parent, spLugs.ptr, M_GREY);
        } else if (brakeMesh && spacer.ptr) k.piece[P_SPACER] = makePiece(veh.ptr, parent, spacer.ptr, M_GREY);
        bool armSplit = suspMesh && armBall.ptr && armLink.ptr && armBush.ptr;
        k.piece[P_ARM] = makePiece(veh.ptr, parent, armSplit ? armBall.ptr : suspMesh ? arm.ptr : cube.ptr, M_GREY);
        if (armSplit) {
            k.piece[P_LINK1] = makePiece(veh.ptr, parent, armLink.ptr, M_GREY);
            k.piece[P_LINK2] = makePiece(veh.ptr, parent, armLink.ptr, M_GREY);
            k.piece[P_BUSH1] = makePiece(veh.ptr, parent, armBush.ptr, M_GREY);
            k.piece[P_BUSH2] = makePiece(veh.ptr, parent, armBush.ptr, M_GREY);
            setMat(k.piece[P_BUSH1], mats[M_BLACK], 1);
            setMat(k.piece[P_BUSH2], mats[M_BLACK], 1);
        }
        bool joints = suspMesh && shStub.ptr && cvBoot.ptr && cvInner.ptr;  // stub, outer joint, tube, inner joint
        if (k.driven) k.piece[P_SHAFT] = makePiece(veh.ptr, parent, joints ? shStub.ptr : suspMesh ? halfShaft.ptr : cylinder.ptr, M_BLACK);
        if (k.driven && suspMesh) {  // the stub on the axle, the outer joint where the shaft bends off, the inner one
            k.piece[P_SHAFT2] = makePiece(veh.ptr, parent, shStub.ptr ? shStub.ptr : halfShaft.ptr, M_BLACK);
            bool modular = shStub.ptr && cvBall.ptr && cvRubber.ptr;  // a ball per joint, a boot along each shaft
            if (modular) {
                k.piece[P_BOOT] = makePiece(veh.ptr, parent, cvBall.ptr, M_BLACK);
                k.piece[P_CVIN] = makePiece(veh.ptr, parent, cvBall.ptr, M_BLACK);
                for (int b : {P_BOOTW, P_BOOTC, P_BOOTI}) k.piece[b] = makePiece(veh.ptr, parent, cvRubber.ptr, M_BLACK);
                if (k.piece[P_SHAFT].ptr) { api->process_event(k.piece[P_SHAFT].ptr, fDestroy, &k.piece[P_SHAFT].ptr); }
                k.piece[P_SHAFT] = makePiece(veh.ptr, parent, shStub.ptr, M_BLACK);
            } else if (joints) {
                k.piece[P_BOOT] = makePiece(veh.ptr, parent, cvBoot.ptr, M_BLACK);
                k.piece[P_CVIN] = makePiece(veh.ptr, parent, cvInner.ptr, M_BLACK);
            }
        }
        k.lastLoc = {1e9, 0, 0};
        car.corners.push_back(k);
    }
    std::vector<double> xs;  // the axles, front first: wheels within 15 cm of each other along the car share one
    for (auto& k : car.corners) {
        bool known = false;
        for (double x : xs) known = known || fabs(x - k.rest.x) < 15;
        if (!known) xs.push_back(k.rest.x);
    }
    std::sort(xs.begin(), xs.end(), [](double a, double b) { return a > b; });
    car.axles = (int)xs.size() < 8 ? (int)xs.size() : 8;
    for (auto& k : car.corners) {
        k.axle = 0;
        for (int i = 0; i < car.axles; i++) if (fabs(xs[i] - k.rest.x) < 15) k.axle = i;
        if (k.driven) car.axleDriven[k.axle] = true;
    }
    if (oId >= 0) car.id = get<long long>(veh.ptr + oId);
    car.bornAt = GetTickCount();
    MT_LOG("decorated %s: %d of %d wheels", objName(classOf(veh.ptr)).c_str(), (int)car.corners.size(), n);
    cars.push_back(std::move(car));
}

static void strip(Car& car) {
    for (auto& k : car.corners)
        for (auto& p : k.piece)
            if (usable(p)) api->process_event(p.ptr, fDestroy, &p.ptr);
    car.corners.clear();
}

static V camera(uintptr_t context) {
    struct { uintptr_t context; int index; uintptr_t ret; } p = {context, 0, 0};
    api->process_event(gameplayCDO, fCamMgr, &p);
    V at{};
    if (p.ret) api->process_event(p.ret, fCamLoc, &at);
    return at;
}

static void evidence(Car& car) {
    for (auto& k : car.corners) {
        if (!usable(k.wheel) || !usable(k.piece[P_DISC])) continue;
        V hub = readV(k.wheel.ptr + oRelLoc), wWorld{}, dWorld{}, sWorld{};
        api->process_event(k.wheel.ptr, fCompLoc, &wWorld);
        api->process_event(k.piece[P_DISC].ptr, fCompLoc, &dWorld);
        api->process_event(k.piece[P_SHOCK].ptr, fCompLoc, &sWorld);
        int drawn = 0;
        for (auto& pc : k.piece) {
            struct { float tolerance; bool ret; } r = {0.5f, false};
            if (usable(pc)) api->process_event(pc.ptr, fRendered, &r);
            drawn += r.ret;
        }
        MT_LOG("  %s: %d/%d pieces drawn | hub %.1f %.1f %.1f rest z %.1f -> up %.1f | steer %.1f deg | R %.1f W %.1f | disc %.1f cm off the hub (world) | shock centre %.1f cm off | spacer %.1f cm",
               objName(k.wheel.ptr).c_str(), drawn, NPIECE, hub.x, hub.y, hub.z, k.rest.z, hub.z - k.rest.z, steerOf(k.wheel.ptr) * 180 / PI, k.R, k.W,
               len(sub(dWorld, wWorld)), len(sub(sWorld, wWorld)), k.spacer);
    }
}

static long long tickTicks, tickCount;  // the tick's own cost, for the log
static LARGE_INTEGER qpcHz;

// ---- the Tuning screen rows (the Custom Plates / ECU Tuning pattern: a menu button, then W_Setting spin rows) ----
struct FitRow { uintptr_t widget, spin, value; };
static struct {
    Ref screen; uintptr_t button, inner, veh; bool held, open; int labelFrames, axle; long long id; DWORD dirty;
    FitRow rows[NFIT]; std::vector<uintptr_t> hidden;
    uintptr_t copyBtn, pasteBtn, copyInner, pasteInner; bool copyHeld, pasteHeld; int btnFrames;
} fp;
static int clip[NFIT]; static bool haveClip;  // an axle's setup, to paste on another axle or another car
static std::mutex builtLock;
static std::vector<Ref> builtScreens, screens;
static uintptr_t cScreen, cSetting, cButton, cButtonBase, cTextBlock, widgetCDO;
static uintptr_t fCreate, fAddChild, fChildIndex, fChildAt, fChildCount, fSetText, fSetRich, fPressed, fRemove, fSetVis, fIsVisible, fPadding;
static int oScreenVeh = -1, oBox, oRowName, oRowSpin, oSpinText, oSpinValue, oBtnName, oBtnRich, oBtnText, oBtnInner;
static void (__fastcall* spinRange)(uintptr_t self, float min, float max, float step, bool snap);
static void (__fastcall* spinSet)(uintptr_t self, float value);
static bool canTune;
static const wchar_t* const BUTTON = L"Brakes & Suspension";

static int onScreenBuilt(uintptr_t obj, uintptr_t, void*, void*) {
    if (cScreen && obj && api->is_a(classOf(obj), cScreen)) { std::lock_guard<std::mutex> g(builtLock); builtScreens.push_back(refOf(obj)); }
    return 0;
}
static bool uiFlag(uintptr_t obj, uintptr_t fn) { unsigned char p[0x10] = {0}; api->process_event(obj, fn, p); return p[0] != 0; }
static bool isA(uintptr_t o, uintptr_t cls) { return o && api->is_a(classOf(o), cls); }
static void uiText(uintptr_t block, const wchar_t* text) {
    unsigned char p[0x40] = {0};
    FStr t = str(text);
    memcpy(p + pTextIn, &t, sizeof t);
    api->process_event(textCDO, fToText, p);
    api->process_event(block, isA(block, cTextBlock) ? fSetText : fSetRich, p + pTextRet);
}
static uintptr_t uiCreate(uintptr_t screen, uintptr_t cls, uintptr_t box) {
    struct { uintptr_t context, cls, player, ret; } c = {screen, cls, 0, 0};
    api->process_event(widgetCDO, fCreate, &c);
    if (!c.ret) return 0;
    struct { uintptr_t content, slot; } add = {c.ret, 0};
    api->process_event(box, fAddChild, &add);
    float pad[4] = {5, 5, 5, 5};
    if (add.slot) api->process_event(add.slot, fPadding, pad);
    return c.ret;
}
static int uiIndex(uintptr_t box, uintptr_t child) {
    struct { uintptr_t content; int ret; } at = {child, -1};
    if (child) api->process_event(box, fChildIndex, &at);
    return at.ret;
}
static void uiShow(uintptr_t w, bool on) { uint8_t v = on ? 0 : 1; api->process_event(w, fSetVis, &v); }
// The spin box scales its step by the value's magnitude, so it holds a step count in 1.00 .. 9.99.
static float toSpin(int v) { return (float)(1 + 0.01 * v); }
static int fromSpin(float v) { return (int)floor((v - 1) / 0.01 + 0.5); }
static const Car* carOfId(long long id) {
    for (auto& c : cars) if (c.id == id) return &c;
    return nullptr;
}
static int axleTop() { const Car* c = carOfId(fp.id); return c && c->axles > 0 ? c->axles - 1 : 1; }
static int fitValue(int i) { return i == F_AXLE ? fp.axle : fitOf(fp.id).v[fp.axle][i]; }
static void fitShow(int i) {
    wchar_t t[48];
    if (i == F_AXLE) {
        const Car* c = carOfId(fp.id);
        bool driven = c && fp.axle < 8 && c->axleDriven[fp.axle];
        swprintf_s(t, L"%d%s", fp.axle + 1, driven ? L" *" : L"");
    }
    else if (FIT[i].unit[0] == L'x') swprintf_s(t, L"x%.2f", fitNum(fitOf(fp.id), fp.axle, i));
    else swprintf_s(t, L"%+.1f cm", fitNum(fitOf(fp.id), fp.axle, i));
    uiText(fp.rows[i].value, t);
}
static uintptr_t fitButton(uintptr_t screen, uintptr_t box, const wchar_t* label) {
    uintptr_t b = uiCreate(screen, cButton, box);
    if (!b) return 0;
    unsigned char p[0x40] = {0};
    FStr t = str(label);
    memcpy(p + pTextIn, &t, sizeof t);
    api->process_event(textCDO, fToText, p);
    wr(b + oBtnText, p + pTextRet, 16);  // moved in raw: nothing destroys the buffer
    return b;
}
static void fitOpen(uintptr_t screen, uintptr_t box) {
    int count = 0;
    api->process_event(box, fChildCount, &count);
    fp.hidden.clear();
    for (int i = 0; i < count; i++) {
        struct { int index; uintptr_t ret; } at = {i, 0};
        api->process_event(box, fChildAt, &at);
        if (at.ret) { uiShow(at.ret, false); fp.hidden.push_back(at.ret); }
    }
    for (int i = 0; i < NFIT; i++) {
        uintptr_t w = uiCreate(screen, cSetting, box);
        uintptr_t spin = w ? get<uintptr_t>(w + oRowSpin) : 0;
        if (!spin) { MT_LOG("could not create the Brakes & Suspension rows"); return; }
        fp.rows[i] = {w, spin, get<uintptr_t>(spin + oSpinText)};
        uiText(get<uintptr_t>(w + oRowName), FIT[i].label);
        spinRange(spin, 1, toSpin(i == F_AXLE ? axleTop() : FIT[i].top), 0.01f, true);
        spinSet(spin, toSpin(fitValue(i)));
        fitShow(i);
    }
    fp.copyBtn = fitButton(screen, box, L"Copy axle");
    fp.pasteBtn = fitButton(screen, box, L"Paste axle");
    fp.copyInner = fp.copyBtn ? get<uintptr_t>(fp.copyBtn + oBtnInner) : 0;
    fp.pasteInner = fp.pasteBtn ? get<uintptr_t>(fp.pasteBtn + oBtnInner) : 0;
    fp.copyHeld = fp.pasteHeld = false;
    fp.btnFrames = 60;
    fp.open = true;
}
static void fitClose() {
    for (uintptr_t b : {fp.copyBtn, fp.pasteBtn}) if (b) api->process_event(b, fRemove, nullptr);
    fp.copyBtn = fp.pasteBtn = 0;
    for (auto& r : fp.rows) if (r.widget) api->process_event(r.widget, fRemove, nullptr);
    for (uintptr_t w : fp.hidden) uiShow(w, true);
    fp.hidden.clear();
    memset(fp.rows, 0, sizeof fp.rows);
    fp.open = false;
}
static void saveFit() {
    saveFitIni(fp.id);
    if (fp.id <= 0 || !fp.veh || oId < 0 || get<long long>(fp.veh + oId) != fp.id) return;
    const Car* c = nullptr;
    for (auto& x : cars) if (x.id == fp.id) c = &x;
    sendFit(fp.veh, fitOf(fp.id), c ? c->axles : 2);
}
static void refit(long long id) {
    for (auto& c : cars)
        if (c.id == id) for (auto& k : c.corners) { k.lastLoc.x = 1e9; k.damperZ = 0; k.linkRun[0] = k.linkRun[1] = k.shaftRun = 0; }
}
static void tickFit(DWORD now) {
    {
        std::lock_guard<std::mutex> g(builtLock);
        for (auto& r : builtScreens)
            if (std::none_of(screens.begin(), screens.end(), [&](const Ref& s) { return s.ptr == r.ptr; })) screens.push_back(r);
        builtScreens.clear();
    }
    if (!cButton) {
        cSetting = api->find_object("WidgetBlueprintGeneratedClass", "W_Setting_C", "/Game/UI/Component/W_Setting");
        cButton = api->find_object("WidgetBlueprintGeneratedClass", "W_Button_C", "/Game/UI/Button/W_Button");
        if (!cSetting) cButton = 0;
        if (!cButton) return;
    }
    if (fp.dirty && now - fp.dirty > 800) { saveFit(); fp.dirty = 0; }
    screens.erase(std::remove_if(screens.begin(), screens.end(), [](const Ref& r) { return !usable(r); }), screens.end());
    uintptr_t screen = 0, box = 0;
    for (auto& r : screens) {
        uintptr_t b = get<uintptr_t>(r.ptr + oBox);
        if (b && uiFlag(r.ptr, fIsVisible)) { screen = r.ptr; box = b; break; }
    }
    if (!screen) { if (fp.open && usable(fp.screen)) fitClose(); fp.open = false; return; }
    uintptr_t vehicle = get<uintptr_t>(screen + oScreenVeh);
    long long id = vehicle && oId >= 0 ? get<long long>(vehicle + oId) : 0;
    if (id <= 0) return;
    if (!fp.dirty) fp.veh = vehicle;
    if (fp.screen.ptr != screen) {
        if (fp.dirty) { saveFit(); fp.dirty = 0; }
        if (fp.open && usable(fp.screen)) fitClose();
        fp.screen = refOf(screen); fp.button = 0; fp.open = false; fp.hidden.clear();
    }
    if (fp.open) {
        if (uiIndex(box, fp.rows[0].widget) < 0) { fp.open = false; fp.button = 0; fp.hidden.clear(); memset(fp.rows, 0, sizeof fp.rows); return; }
        if (fp.id != id) {  // another car on the same screen
            if (fp.dirty) { saveFit(); fp.dirty = 0; }
            fp.id = id;
            for (int i = 0; i < NFIT; i++) { spinSet(fp.rows[i].spin, toSpin(fitValue(i))); fitShow(i); }
            return;
        }
        if (fp.btnFrames > 0) {  // the game writes its own label over a new button for a few frames
            fp.btnFrames--;
            for (auto [b, l] : {std::pair<uintptr_t, const wchar_t*>{fp.copyBtn, L"Copy axle"}, {fp.pasteBtn, haveClip ? L"Paste axle" : L"Paste axle (copy one first)"}})
                if (b) for (int at : {oBtnName, oBtnRich}) if (uintptr_t t = get<uintptr_t>(b + at)) uiText(t, l);
        }
        auto clickedUp = [](uintptr_t inner, bool& held) { bool down = inner && uiFlag(inner, fPressed), up = held && !down; held = down; return up; };
        if (clickedUp(fp.copyInner, fp.copyHeld)) {
            memcpy(clip, fitOf(id).v[fp.axle], sizeof clip);
            haveClip = true; fp.btnFrames = 2;
            MT_LOG("Brakes & Suspension: axle %d copied", fp.axle + 1);
        }
        if (clickedUp(fp.pasteInner, fp.pasteHeld) && haveClip) {
            memcpy(fitOf(id).v[fp.axle] + 1, clip + 1, sizeof(int) * (NFIT - 1));
            fp.dirty = now;
            refit(id);
            for (int r = 0; r < NFIT; r++) { spinSet(fp.rows[r].spin, toSpin(fitValue(r))); fitShow(r); }
            MT_LOG("Brakes & Suspension: pasted onto axle %d", fp.axle + 1);
            return;
        }
        for (int i = 0; i < NFIT; i++) {
            int v = fromSpin(get<float>(fp.rows[i].spin + oSpinValue));
            int top = i == F_AXLE ? axleTop() : FIT[i].top;
            v = v < 0 ? 0 : v > top ? top : v;
            if (v == fitValue(i)) { fitShow(i); continue; }  // held at the end of its range the spin box writes its own number
            if (i == F_AXLE) {
                fp.axle = v;
                for (int r = 0; r < NFIT; r++) { spinSet(fp.rows[r].spin, toSpin(fitValue(r))); fitShow(r); }
                return;
            }
            fitOf(id).v[fp.axle][i] = v;
            fp.dirty = now;
            if (loggingOn()) MT_LOG("fit axle %d %ls = %d (%.1f)", fp.axle + 1, FIT[i].key, v, fitNum(fitOf(id), fp.axle, i));
            fitShow(i);
            refit(id);
        }
        return;
    }
    if (uiIndex(box, fp.button) >= 0) {
        if (fp.labelFrames > 0) {
            fp.labelFrames--;
            for (int at : {oBtnName, oBtnRich}) if (uintptr_t b = get<uintptr_t>(fp.button + at)) uiText(b, BUTTON);
        }
        bool down = uiFlag(fp.inner, fPressed), up = fp.held && !down;
        fp.held = down;
        if (up) { fp.id = id; if (fp.axle > axleTop()) fp.axle = 0; fitOpen(screen, box); }
        return;
    }
    struct { int index; uintptr_t ret; } first = {0, 0};
    api->process_event(box, fChildAt, &first);
    if (!isA(first.ret, cButtonBase) || !uiFlag(first.ret, fIsVisible)) return;  // not the menu, or another plugin's rows are open
    // a button made before and still in the list: keep it rather than add a second
    static std::vector<Ref> made;
    for (auto& r : made)
        if (usable(r) && uiIndex(box, r.ptr) >= 0) { fp.button = r.ptr; fp.inner = get<uintptr_t>(r.ptr + oBtnInner); fp.held = false; return; }
    made.erase(std::remove_if(made.begin(), made.end(), [](const Ref& r) { return !usable(r); }), made.end());
    fp.button = uiCreate(screen, cButton, box);
    if (!fp.button) return;
    made.push_back(refOf(fp.button));
    unsigned char p[0x40] = {0};
    FStr t = str(BUTTON);
    memcpy(p + pTextIn, &t, sizeof t);
    api->process_event(textCDO, fToText, p);
    wr(fp.button + oBtnText, p + pTextRet, 16);  // moved in raw: nothing destroys the buffer
    fp.labelFrames = 60;
    fp.inner = get<uintptr_t>(fp.button + oBtnInner);
    fp.held = false;
    MT_LOG("Brakes & Suspension button made");
}

static void tick(void*) {
    if (lock.try_lock()) {
        if (!found.empty()) { candidates.swap(found); found.clear(); rests.swap(foundRest); foundRest.clear(); }
        lock.unlock();
    }
    for (size_t i = 0; i < cars.size();) {  // a vehicle that is gone took its pieces with it
        if (!usable(cars[i].veh)) { cars[i] = std::move(cars.back()); cars.pop_back(); continue; }
        i++;
    }
    static DWORD pickAt, logAt;
    DWORD now = GetTickCount();
    if ((int)(now - pickAt) >= 0 && !candidates.empty()) {
        pickAt = now + 500;
        // the MaxVehicles nearest the camera within Range; further than 1.15 x Range loses its set
        static std::vector<std::pair<double, int>> nearby;
        nearby.clear();
        V cam{};
        bool haveCam = false;
        for (int i = 0; i < (int)candidates.size(); i++) {
            if (!usable(candidates[i])) continue;
            if (!haveCam) { cam = camera(candidates[i].ptr); haveCam = true; }
            V at{};
            api->process_event(candidates[i].ptr, fLoc, &at);
            double d = len(sub(at, cam));
            if (d < kRange * 115.0) nearby.push_back({d, i});
        }
        std::sort(nearby.begin(), nearby.end());
        if ((int)nearby.size() > kMax) nearby.resize(kMax);
        for (auto& c : cars) {
            bool keep = false;
            for (auto& n : nearby) keep = keep || candidates[n.second].ptr == c.veh.ptr;
            if (!keep) c.veh.ptr = 0;  // dropped below
        }
        for (size_t i = 0; i < cars.size();) {
            if (!cars[i].veh.ptr) {
                for (auto& k : cars[i].corners) for (auto& p : k.piece) if (usable(p)) api->process_event(p.ptr, fDestroy, &p.ptr);
                cars[i] = std::move(cars.back()); cars.pop_back(); continue;
            }
            i++;
        }
        for (auto& n : nearby) {  // one new vehicle per pass, the nearest first
            if (n.first > kRange * 100.0) break;
            bool have = false;
            for (auto& c : cars) have = have || c.veh.ptr == candidates[n.second].ptr;
            if (have) continue;
            gStep = "decorate"; if (assets(candidates[n.second].ptr)) decorate(candidates[n.second]);
            break;
        }
        if (loggingOn() && (int)(now - logAt) >= 0 && !cars.empty() && !nearby.empty()) {
            logAt = now + 3000;
            for (auto& c : cars)
                if (c.veh.ptr == candidates[nearby[0].second].ptr) {
                    gWatch = c.veh.ptr;
                    if (!qpcHz.QuadPart) QueryPerformanceFrequency(&qpcHz);
                    MT_LOG("nearest, %s at %.0f m; %d vehicles decorated, tick %.3f ms average", objName(classOf(c.veh.ptr)).c_str(), nearby[0].first / 100,
                           (int)cars.size(), tickCount ? 1000.0 * tickTicks / tickCount / qpcHz.QuadPart : 0.0);
                    tickTicks = tickCount = 0;
                    evidence(c);
                }
        }
    }
    // sync probe: does the wheel's RelativeLocation move as often as the wheel is drawn moving?
    if (loggingOn() && !cars.empty() && !cars[0].corners.empty() && usable(cars[0].corners[0].wheel)) {
        static DWORD probeAt; static int nTick, nRel, nWorld; static V lastRel, lastW; static double worst;
        uintptr_t w = cars[0].corners[0].wheel.ptr, par = get<uintptr_t>(w + oParent);
        V rel = readV(w + oRelLoc), ww{}, pw{};
        api->process_event(w, fCompLoc, &ww);
        if (par) api->process_event(par, fCompLoc, &pw);
        V dw = sub(ww, pw);
        nTick++;
        if (len(sub(rel, lastRel)) > 0.01) nRel++;
        if (len(sub(dw, lastW)) > 0.01) nWorld++;
        double gap = fabs(len(dw) - len(rel));
        if (gap > worst) worst = gap;
        lastRel = rel; lastW = dw;
        if ((int)(now - probeAt) >= 0) {
            probeAt = now + 1000;
            MT_LOG("sync %s: %d ticks/s, wheel RelativeLocation moved %d, world offset moved %d, worst |world|-|rel| %.2f cm, rel z %.2f",
                   objName(w).c_str(), nTick, nRel, nWorld, worst, rel.z);
            nTick = nRel = nWorld = 0; worst = 0;
        }
    }
    gStep = "tuning panel"; if (canTune) tickFit(now);  // every tick: at 33 ms a quick click fell between two looks (2026-11-09)
    for (auto& c : cars) if (!c.id && oId >= 0) c.id = get<long long>(c.veh.ptr + oId);
    static DWORD kitAt;
    bool kits = (int)(now - kitAt) >= 0;
    if (kits) kitAt = now + 1000;
    for (auto& c : cars) {
        if (kits && !c.corners.empty()) {  // a kit fitted or swapped in the garage
            bool settled = (int)(now - c.bornAt) >= kSettleMs;
            gStep = "paint slots"; if (kPaintSlots && settled) addPaintSlots(c.veh.ptr);
            gStep = "body colours"; if (kPaintSlots && settled) addBodyColors(c.veh.ptr);
            gStep = "fit read";
            if (c.id && !(fp.dirty && fp.id == c.id)) {
                Fit f{};
                bool carries = readCarFit(c.veh.ptr, f);
                if (!carries && tunedHere(c.id) && (int)(now - c.resentAt) >= 0) {  // lost on load: put it back
                    c.resentAt = now + 5000;
                    sendFit(c.veh.ptr, fitOf(c.id), c.axles);
                    for (auto& k : c.corners) k.lastLoc.x = 1e9;
                } else if (carries && memcmp(&f, &fitOf(c.id), sizeof f) != 0) {
                    fits[c.id] = f;
                    for (auto& k : c.corners) k.lastLoc.x = 1e9;
                    if (fp.open && fp.id == c.id) for (int i = 0; i < NFIT; i++) { spinSet(fp.rows[i].spin, toSpin(fitValue(i))); fitShow(i); }
                }
            }
            gStep = "kit"; std::string key = fittedBrake(c.veh.ptr);
            uint32_t paint[NPAINT];
            readPaint(c.veh.ptr, paint);
            bool repaint = key != c.kit || memcmp(paint, c.paint, sizeof paint) != 0;
            if (key != c.kit) {
                readKit(c, key);
                for (auto& k : c.corners) k.lastLoc.x = 1e9;
                MT_LOG("%s: brakes %s -> disc %d mm, caliper %d", objName(classOf(c.veh.ptr)).c_str(), key.empty() ? "(none)" : key.c_str(), c.kitMm, c.kitMat);
            }
            gStep = "repaint";
            if (repaint) {  // a painted slot gets this car's own material; an unpainted one the kit's or the default
                memcpy(c.paint, paint, sizeof paint);
                static const int dflt[NPAINT] = {-1, M_DISC, M_GREY, -1, M_GREY, M_GREY, M_GREY, M_BLACK};
                for (auto& k : c.corners) {
                    Ref use[NPAINT];
                    for (int i = 0; i < NPAINT; i++) {
                        if (paint[i] && i < 8) {
                            if (!usable(k.mid[i])) { k.mid[i] = newMid(c.veh.ptr, COLOURS[M_GREY]); k.dirtAt = 0; }
                            setColour(k.mid[i], paint[i]);
                            use[i] = k.mid[i];
                        } else use[i] = mats[i == PC_CALIPER ? c.kitMat : dflt[i] < 0 ? M_GREY : dflt[i]];
                    }
                    setMat(k.piece[P_CALIPER], use[PC_CALIPER]);
                    setMat(k.piece[P_DISC], use[PC_DISC]);
                    setMat(k.piece[P_DISC], use[PC_DISC], 1);
                    setMat(k.piece[P_DISC], use[PC_LUGS], 2);
                    setMat(k.piece[P_SPACER], use[PC_SPACER]);
                    if (usable(k.piece[P_SPLUGS])) setMat(k.piece[P_SPLUGS], use[PC_LUGS]);
                    else setMat(k.piece[P_SPACER], use[PC_LUGS], 1);
                    setMat(k.piece[P_KNUCKLE], use[PC_ARMS]);
                    setMat(k.piece[P_WEB], use[PC_ARMS]);
                    setMat(k.piece[P_WEB2], use[PC_ARMS]);
                    setMat(k.piece[P_MOUNTS], use[PC_ARMS]);
                    setMat(k.piece[P_MOUNTS2], use[PC_ARMS]);
                    setMat(k.piece[P_ARM], use[PC_ARMS]);
                    for (int pc : {P_LINK1, P_LINK2, P_BUSH1, P_BUSH2}) setMat(k.piece[pc], use[PC_ARMS]);
                    setMat(k.piece[P_SHAFT], use[PC_SHAFT]);
                    setMat(k.piece[P_SHAFT2], use[PC_SHAFT]);
                    setMat(k.piece[P_BOOT], use[PC_SHAFT], 1);
                    setMat(k.piece[P_CVIN], use[PC_SHAFT], 1);
                    if (usable(k.piece[P_BOOTW])) {  // the balls are all metal; the boots' clamps too, their rubber stays black
                        setMat(k.piece[P_BOOT], use[PC_SHAFT]); setMat(k.piece[P_CVIN], use[PC_SHAFT]);
                        for (int b : {P_BOOTW, P_BOOTC, P_BOOTI}) setMat(k.piece[b], use[PC_SHAFT], 1);
                    }
                    if (suspMesh) {  // SM_Coilover's slot 0 is "Spring", slot 1 "Damper"
                        setMat(k.piece[P_SHOCK], use[PC_SUSP], 0);
                        setMat(k.piece[P_SHOCK], paint[PC_DAMPER] ? use[PC_DAMPER] : mats[M_YELLOW], 1);
                        setMat(k.piece[P_CBOT], paint[PC_DAMPER] ? use[PC_DAMPER] : mats[M_YELLOW]);
                        setMat(k.piece[P_DBODY], paint[PC_DAMPER] ? use[PC_DAMPER] : mats[M_YELLOW]);
                        setMat(k.piece[P_DROD], paint[PC_DAMPER] ? use[PC_DAMPER] : mats[M_YELLOW]);
                        setMat(k.piece[P_CTOP], paint[PC_DAMPER] ? use[PC_DAMPER] : mats[M_YELLOW]);
                    } else if (!k.prop) setMat(k.piece[P_SHOCK], paint[PC_DAMPER] ? use[PC_DAMPER] : mats[M_BODY]);  // the prop's one section keeps its own look
                }
                if (loggingOn()) {
                    std::string names;
                    if (oBodyColors >= 0) {
                        uintptr_t data = get<uintptr_t>(c.veh.ptr + oBodyColors);
                        int n = get<int>(c.veh.ptr + oBodyColors + 8);
                        char b[96];
                        for (int e = 0; data && e < n && e < 40; e++)
                            if (api->fname(data + (uintptr_t)e * colSize + oColSlot, b, sizeof b)) { names += ' '; names += b; }
                    }
                    MT_LOG("%s: paint caliper %08x disc %08x suspension %08x dampers %08x | body colours:%s", objName(classOf(c.veh.ptr)).c_str(),
                           paint[0], paint[1], paint[2], paint[3], names.c_str());  // spacer/lugs: see "colours now"
                }
            }
        }
        gStep = "dirt";
        if (kits && kDirt && brakeMesh && dirtReady())
            for (auto& k : c.corners) {
                if (!usable(k.wheel) || (k.dirtAt && (int)(now - k.dirtAt) < 3000)) continue;
                bool first = !k.dirtAt;
                k.dirtAt = now | 1;
                uintptr_t src = dirtSource(k.wheel.ptr);
                int n = 0;
                for (int i = 0; i < NPAINT && i < 8; i++) n += copyDirt(src, k.mid[i]);
                static int told;
                if (first && told < 6) { told++; MT_LOG("%s %s: dirt %s (%d parameters)", objName(classOf(c.veh.ptr)).c_str(), objName(k.wheel.ptr).c_str(), src ? "from its wheel" : "-- the wheel has none", n); }
            }
        for (auto& k : c.corners) {
            if (!usable(k.wheel)) continue;
            V hub = readV(k.wheel.ptr + oRelLoc), sc = readV(k.wheel.ptr + oRelScale);
            double steer = steerOf(k.wheel.ptr);
            if (len(sub(sc, k.lastScale)) > 1e-4) { k.lastScale = sc; measure(k, sc); k.lastLoc.x = 1e9; }  // wheel size tuned
            double rr[3]; rd(k.wheel.ptr + oRelRot, rr, sizeof rr);  // Pitch, Yaw, Roll: the wheel's spin is its pitch
            k.spin = kSpin ? -rr[0] * PI / 180 : 0;
            if (len(sub(hub, k.lastLoc)) < 0.05 && fabs(steer - k.lastSteer) < 0.002 && fabs(k.spin - k.lastSpin) < 0.004) continue;
            k.lastLoc = hub; k.lastSteer = steer; k.lastSpin = k.spin;
            gStep = "pose"; pose(k, hub, steer, c.kitMm, geoOf(c.id, k.axle));
        }
    }
}

// ---- the guard (no C++ objects in here, so it can hold __try) ----
static const int MAX_FAULTS = 5;
static void* faultAt; static uintptr_t faultRead; static unsigned faultCode;
static volatile LONG faults, disabled, queued;
static DWORD pauseUntil;
static int faultFilter(unsigned code, EXCEPTION_POINTERS* ep) {
    if (ep && ep->ExceptionRecord) {
        faultAt = ep->ExceptionRecord->ExceptionAddress; faultCode = code;
        faultRead = ep->ExceptionRecord->NumberParameters > 1 ? (uintptr_t)ep->ExceptionRecord->ExceptionInformation[1] : 0;
    }
    return code == EXCEPTION_ACCESS_VIOLATION || code == EXCEPTION_IN_PAGE_ERROR || code == EXCEPTION_DATATYPE_MISALIGNMENT
        ? EXCEPTION_EXECUTE_HANDLER : EXCEPTION_CONTINUE_SEARCH;
}
static void logFault(LONG n) {
    HMODULE mod = nullptr; char name[MAX_PATH] = "?";
    if (GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT, (LPCSTR)faultAt, &mod) && mod)
        GetModuleFileNameA(mod, name, MAX_PATH);
    const char* base = strrchr(name, '\\') ? strrchr(name, '\\') + 1 : name;
    MT_LOG("FAULT %ld of %d in step '%s': %08x at %s+0x%llx, address 0x%llx -- retrying in 500 ms", n, MAX_FAULTS, gStep, faultCode, base,
           (unsigned long long)((uintptr_t)faultAt - (uintptr_t)mod), (unsigned long long)faultRead);
}
static void forget() { for (auto& c : cars) strip(c); cars.clear(); }
static void guardedTick(void* p) {
    InterlockedExchange(&queued, 0);
    if (disabled || (int)(GetTickCount() - pauseUntil) < 0) return;
    LARGE_INTEGER a, b;
    QueryPerformanceCounter(&a);
    __try {
        tick(p);
    } __except (faultFilter(GetExceptionCode(), GetExceptionInformation())) {
        LONG n = InterlockedIncrement(&faults);
        pauseUntil = GetTickCount() + 500;
        logFault(n);
        if (n >= MAX_FAULTS) { InterlockedExchange(&disabled, 1); MT_LOG("too many faults: Suspension Tuning stops for this session"); }
        __try { forget(); } __except (EXCEPTION_EXECUTE_HANDLER) { InterlockedExchange(&disabled, 1); }
        return;
    }
    QueryPerformanceCounter(&b);
    tickTicks += b.QuadPart - a.QuadPart; tickCount++;
}

// The tick is queued from a thread of its own: the scan below takes about a second, and queuing from the
// scanning thread froze every piece for that second, then snapped it to the wheel (the "two syncs a second").
// Probe (2026-11-09): every change to the nearest car's BodyColors, as it happens, to see what the Paint screen does.
static DWORD WINAPI paintWatch(LPVOID) {
    std::string last;
    for (;;) {
        Sleep(16);
        uintptr_t v = gWatch.load();
        if (!v || oBodyColors < 0) continue;
        uintptr_t a[2] = {};
        if (!api->read(v + oBodyColors, a, sizeof a)) continue;
        int n = (int)(a[1] & 0xffffffff);
        if (!a[0] || n <= 0 || n > 24) continue;
        unsigned char buf[24 * 32];
        if (colSize > 32 || !api->read(a[0], buf, (size_t)n * colSize)) continue;
        std::string cur;
        char b[96], h[16];
        for (int e = 0; e < n; e++) {
            if (api->fname(a[0] + (uintptr_t)e * colSize + oColSlot, b, sizeof b)) cur += b;
            snprintf(h, sizeof h, "=%08x ", *(uint32_t*)(buf + e * colSize + oColColor)); cur += h;
        }
        if (cur != last) { MT_LOG("colours now: %s", cur.c_str()); last = cur; }
    }
}
static DWORD WINAPI pump(LPVOID) {
    timeBeginPeriod(1);  // a 1 ms Sleep: the tick runs within ~1 ms of the game moving a wheel
    while (!disabled) {
        if (api->game_thread_ready() && !InterlockedCompareExchange(&queued, 1, 0)) api->run_on_game_thread(guardedTick, nullptr);
        Sleep(1);
    }
    return 0;
}

// Probe (2026-11-09): what the Paint screen sends on Confirm -- are the QX_* colours in it?
static int pSendCust = -1, oCustColors = -1;
static int onSendCustom(uintptr_t obj, uintptr_t, void* parms, void*) {
    if (pSendCust < 0 || oCustColors < 0 || !parms) return 0;
    uintptr_t arr = (uintptr_t)parms + pSendCust + oCustColors;
    uintptr_t data = get<uintptr_t>(arr);
    int n = get<int>(arr + 8);
    std::string names;
    char b[96];
    for (int e = 0; data && e < n && e < 40; e++) {
        uintptr_t c = data + (uintptr_t)e * colSize;
        if (api->fname(c + oColSlot, b, sizeof b)) { names += ' '; names += b; char h[16]; snprintf(h, sizeof h, "=%08x", get<uint32_t>(c + oColColor)); names += h; }
    }
    MT_LOG("paint sent (%s): %d colours:%s", objName(classOf(obj)).c_str(), n, names.c_str());
    return 0;
}
static void migrateIni() {
    // renamed from Suspension Visuals (2026-11-09): its ini moves over once, section and all, so setups and paint stay
    std::wstring dir = gameDir() + L"plugins\\", from = dir + L"suspension_visuals.ini", to = dir + L"suspension_tuning.ini";
    if (GetFileAttributesW(to.c_str()) != INVALID_FILE_ATTRIBUTES || GetFileAttributesW(from.c_str()) == INVALID_FILE_ATTRIBUTES) return;
    FILE* f = _wfopen(from.c_str(), L"rb");
    if (!f) return;
    std::string text; char buf[4096]; size_t n;
    while ((n = fread(buf, 1, sizeof buf, f)) > 0) text.append(buf, n);
    fclose(f);
    if (text.size() >= 2 && (unsigned char)text[0] == 0xFF && (unsigned char)text[1] == 0xFE) {  // UTF-16: replace as wide text
        std::wstring w((const wchar_t*)(text.data() + 2), (text.size() - 2) / 2);
        for (size_t at; (at = w.find(L"[SuspensionVisuals]")) != std::wstring::npos;) w.replace(at, 19, L"[SuspensionTuning]");
        text.assign(text.data(), 2);
        text.append((const char*)w.data(), w.size() * 2);
    } else
        for (size_t at; (at = text.find("[SuspensionVisuals]")) != std::string::npos;) text.replace(at, 19, "[SuspensionTuning]");
    if (FILE* g = _wfopen(to.c_str(), L"wb")) { fwrite(text.data(), 1, text.size(), g); fclose(g); }
    MoveFileW(from.c_str(), (from + L".migrated").c_str());
    MT_LOG("settings moved over from suspension_visuals.ini");
}
static DWORD WINAPI run(LPVOID) {
    MT_LOG("Suspension Tuning loaded");
    kRange = iniInt(L"suspension_tuning.ini", L"SuspensionTuning", L"Range", 60);
    kMax = iniInt(L"suspension_tuning.ini", L"SuspensionTuning", L"MaxVehicles", 8);
    kInset = iniNum(L"suspension_tuning.ini", L"SuspensionTuning", L"BrakeInset", 0.25);
    kFlip = iniInt(L"suspension_tuning.ini", L"SuspensionTuning", L"FlipShock", 0) != 0;
    kPaintSlots = iniInt(L"suspension_tuning.ini", L"SuspensionTuning", L"PaintSlots", 1) != 0;
    kDirt = iniInt(L"suspension_tuning.ini", L"SuspensionTuning", L"Dirt", 1) != 0;
    kSpin = iniInt(L"suspension_tuning.ini", L"SuspensionTuning", L"Spin", 1) != 0;
    kSettleMs = 1000 * iniInt(L"suspension_tuning.ini", L"SuspensionTuning", L"SettleSeconds", 10);
    MT_LOG("paint entries %s (after %d s), dirt %s, spin %s", kPaintSlots ? "on" : "OFF", kSettleMs / 1000, kDirt ? "on" : "OFF", kSpin ? "on" : "OFF");
    if (!api->wait_engine(INFINITE)) { MT_LOG("FAILED: no engine"); return 0; }
    cVeh = api->find_object("Class", "MTVehicle", "/Script/MotorTown");
    cWheel = api->find_object("Class", "MHWheelComponent", "/Script/MotorTown");
    cMesh = api->find_object("Class", "StaticMeshComponent", "/Script/Engine");
    uintptr_t cClass = api->find_object("Class", "Class", "/Script/CoreUObject");
    uintptr_t cScene = api->find_object("Class", "SceneComponent", "/Script/Engine"), cActor = api->find_object("Class", "Actor", "/Script/Engine");
    fAdd = api->find_object("Function", "AddComponentByClass", "Actor");
    fFinish = api->find_object("Function", "FinishAddComponent", "Actor");
    fLoc = api->find_object("Function", "K2_GetActorLocation", "Actor");
    fSetMesh = api->find_object("Function", "SetStaticMesh", "StaticMeshComponent");
    fMobility = api->find_object("Function", "SetMobility", "SceneComponent");
    fXf = api->find_object("Function", "K2_SetRelativeTransform", "SceneComponent");
    fAttach = api->find_object("Function", "K2_AttachToComponent", "SceneComponent");
    fCompLoc = api->find_object("Function", "K2_GetComponentLocation", "SceneComponent");
    fCollision = api->find_object("Function", "SetCollisionEnabled", "PrimitiveComponent");
    fShadow = api->find_object("Function", "SetCastShadow", "PrimitiveComponent");
    fSetMaterial = api->find_object("Function", "SetMaterial", "PrimitiveComponent");
    fDestroy = api->find_object("Function", "K2_DestroyComponent", "ActorComponent");
    fRendered = api->find_object("Function", "WasRecentlyRendered", "PrimitiveComponent");
    fMeshBounds = api->find_object("Function", "GetBounds", "StaticMesh");
    fCamMgr = api->find_object("Function", "GetPlayerCameraManager", "GameplayStatics");
    fCamLoc = api->find_object("Function", "GetCameraLocation", "PlayerCameraManager");
    gameplayCDO = api->find_object("GameplayStatics", "Default__GameplayStatics", "/Script/Engine");
    fMakeMid = api->find_object("Function", "CreateDynamicMaterialInstance", "KismetMaterialLibrary");
    matCDO = api->find_object("KismetMaterialLibrary", "Default__KismetMaterialLibrary", "/Script/Engine");
    fMidColor = api->find_object("Function", "SetVectorParameterValue", "MaterialInstanceDynamic");
    fToName = api->find_object("Function", "Conv_StringToName", "KismetStringLibrary");
    stringCDO = api->find_object("KismetStringLibrary", "Default__KismetStringLibrary", "/Script/Engine");
    systemCDO = api->find_object("KismetSystemLibrary", "Default__KismetSystemLibrary", "/Script/Engine");
    fPath = api->find_object("Function", "MakeSoftObjectPath", "KismetSystemLibrary");
    fRef = api->find_object("Function", "Conv_SoftObjPathToSoftObjRef", "KismetSystemLibrary");
    fLoad = api->find_object("Function", "LoadAsset_Blocking", "KismetSystemLibrary");
    uintptr_t need[] = {cClass, cVeh, cWheel, cMesh, cScene, cActor, fAdd, fFinish, fLoc, fSetMesh, fMobility, fXf, fAttach, fCompLoc, fCollision, fShadow, fSetMaterial,
                        fDestroy, fRendered, fMeshBounds, fCamMgr, fCamLoc, gameplayCDO, fMakeMid, matCDO, fMidColor, fToName, stringCDO, systemCDO, fPath, fRef, fLoad};
    for (uintptr_t p : need)
        if (!p) { MT_LOG("FAILED: engine functions not found -- nothing drawn"); return 0; }
    oWheels = off(cVeh, "Wheels"); oRoot = off(cActor, "RootComponent"); oStaticMesh = off(cMesh, "StaticMesh");
    oRelLoc = off(cScene, "RelativeLocation"); oRelRot = off(cScene, "RelativeRotation"); oRelScale = off(cScene, "RelativeScale3D");
    oParent = off(cScene, "AttachParent");
    if (uintptr_t sPart = api->find_object("ScriptStruct", "MTVehiclePart", "/Script/MotorTown")) {
        oParts = off(cVeh, "Net_Parts"); oPartKey = off(sPart, "Key"); oPartSlot = off(sPart, "Slot");
        partSize = get<int>(sPart + L().struct_child_props + 8);
        if (oPartKey < 0 || oPartSlot < 0 || partSize <= 0) oParts = -1;
    }
    if (oParts < 0) MT_LOG("fitted parts not readable -- every brake is the stock one");
    uintptr_t sCust = api->find_object("ScriptStruct", "MTVehicleCustomization", "/Script/MotorTown"),
              sCol = api->find_object("ScriptStruct", "MTVehicleCustomizationColor", "/Script/MotorTown");
    if (sCust && sCol && off(cVeh, "Customization") >= 0 && off(sCust, "BodyColors") >= 0) {
        oBodyColors = off(cVeh, "Customization") + off(sCust, "BodyColors");
        oColSlot = off(sCol, "MaterialSlotName"); oColColor = off(sCol, "Color");
        colSize = get<int>(sCol + L().struct_child_props + 8);
        if (oColSlot < 0 || oColColor < 0 || colSize <= 0) oBodyColors = -1;
    }
    if (oBodyColors < 0) MT_LOG("paint not readable -- brakes and suspension keep their default colours");
    fRightPad = api->find_object("Function", "RightPad", "KismetStringLibrary");
    fToText = api->find_object("Function", "Conv_StringToText", "KismetTextLibrary");
    textCDO = api->find_object("KismetTextLibrary", "Default__KismetTextLibrary", "/Script/Engine");
    uintptr_t sSlot = api->find_object("ScriptStruct", "MTVehicleColorSlot", "/Script/MotorTown"),
              sT2 = api->find_object("ScriptStruct", "MTTextByTexts", "/Script/MotorTown");
    if (oBodyColors >= 0 && fRightPad && fToText && textCDO && sSlot && sT2) {
        oColorSlots = off(cVeh, "ColorSlots");
        int valSize = get<int>(sSlot + L().struct_child_props + 8);
        pairSize = (8 + valSize + 7) & ~7;
        oSlotText = off(sSlot, "DisplayName"); oSlotText2 = off(sSlot, "DisplayName2"); oSlotColor = off(sSlot, "DefaultColor");
        oSlotMetal = off(sSlot, "DefaultMetallic"); oSlotRough = off(sSlot, "DefaultRoughness");
        oT2Pattern = off(sT2, "Pattern"); oT2Override = off(sT2, "FullStringOverride");
        pPadIn = off(fRightPad, "SourceString"); pPadCount = off(fRightPad, "ChCount"); pPadRet = off(fRightPad, "ReturnValue");
        pTextIn = off(fToText, "InString"); pTextRet = off(fToText, "ReturnValue");
        for (int o : {oColorSlots, valSize, oSlotText, oSlotText2, oSlotColor, oSlotMetal, oSlotRough, oT2Pattern, oT2Override, pPadIn, pPadCount, pPadRet, pTextIn, pTextRet})
            if (o < 0) oColorSlots = -1;
        if (valSize != 112) oColorSlots = -1;  // the layout this was written against (0.7.20); anything else, stay out
    }
    MT_LOG(oColorSlots >= 0 ? "paint slots: Brake Caliper, Brake Disc, Suspension and Dampers go on the Paint screen"
                            : "paint slots: layout not found -- no Paint entries");
    oDiffComp = off(cWheel, "DifferentialComponent"); oShaftComp = off(cWheel, "DriveShaftComponent");
    pXf = off(fXf, "NewTransform"); pXfTeleport = off(fXf, "bTeleport");
    for (int o : {oWheels, oRoot, oStaticMesh, oRelLoc, oRelRot, oRelScale, oParent, pXf, pXfTeleport})
        if (o < 0) { MT_LOG("FAILED: layout not found -- nothing drawn"); return 0; }
    MT_LOG("ready: Range %d m, MaxVehicles %d, BrakeInset %.2f", kRange, kMax, kInset);

    if (uintptr_t fSend = api->find_object("Function", "ServerSetVehicleCustomization", "MotorTownPlayerController")) {
        uintptr_t sCust = api->find_object("ScriptStruct", "MTVehicleCustomization", "/Script/MotorTown");
        pSendCust = off(fSend, "Customization"); oCustColors = sCust ? off(sCust, "BodyColors") : -1;
        static const char* kSend[] = {"ServerSetVehicleCustomization"};
        if (loggingOn()) {
            MT_LOG("paint probe %s", api->watch_functions(kSend, 1, onSendCustom, nullptr) ? "on" : "OFF");
            CloseHandle(CreateThread(nullptr, 0, paintWatch, nullptr, 0, nullptr));
        }
    }
    oId = off(cVeh, "Net_VehicleId");
    fSendFit = api->find_object("Function", "ServerSetVehicleCustomization", "MotorTownPlayerController");
    fGetPCFit = api->find_object("Function", "GetPlayerController", "GameplayStatics");
    if (fSendFit) { pSendVeh = off(fSendFit, "Vehicle"); pSendCus = off(fSendFit, "Customization"); }
    oCustom = off(cVeh, "Customization");
    if (uintptr_t sC = api->find_object("ScriptStruct", "MTVehicleCustomization", "/Script/MotorTown")) custSize = get<int>(sC + L().struct_child_props + 8);
    if (!fSendFit || !fGetPCFit || pSendVeh < 0 || pSendCus < 0 || oCustom < 0 || custSize <= 0) MT_LOG("customisation send not found -- Brakes & Suspension settings stay until the game closes");
    cScreen = api->find_object("Class", "GarageTuningWidget", "/Script/MotorTown");
    uintptr_t cChild = api->find_object("Class", "GarageChildWidget", "/Script/MotorTown");
    uintptr_t cRowBase = api->find_object("Class", "SettingWidget", "/Script/MotorTown");
    uintptr_t cSpinRow = api->find_object("Class", "SpinBoxString", "/Script/MotorTown");
    cButtonBase = api->find_object("Class", "MTButtonWidget", "/Script/MotorTown");
    cTextBlock = api->find_object("Class", "TextBlock", "/Script/UMG");
    widgetCDO = api->find_object("WidgetBlueprintLibrary", "Default__WidgetBlueprintLibrary", "/Script/UMG");
    fCreate = api->find_object("Function", "Create", "WidgetBlueprintLibrary");
    fAddChild = api->find_object("Function", "AddChild", "PanelWidget");
    fChildIndex = api->find_object("Function", "GetChildIndex", "PanelWidget");
    fChildAt = api->find_object("Function", "GetChildAt", "PanelWidget");
    fChildCount = api->find_object("Function", "GetChildrenCount", "PanelWidget");
    fSetText = api->find_object("Function", "SetText", "TextBlock");
    fSetRich = api->find_object("Function", "SetText", "RichTextBlock");
    fPressed = api->find_object("Function", "IsPressed", "Button");
    fRemove = api->find_object("Function", "RemoveFromParent", "Widget");
    fSetVis = api->find_object("Function", "SetVisibility", "Widget");
    fIsVisible = api->find_object("Function", "IsVisible", "Widget");
    fPadding = api->find_object("Function", "SetPadding", "VerticalBoxSlot");
    canTune = oId >= 0 && cScreen && cChild && cRowBase && cSpinRow && cButtonBase && cTextBlock && widgetCDO && fCreate && fAddChild && fChildIndex &&
              fChildAt && fChildCount && fSetText && fSetRich && fPressed && fRemove && fSetVis && fIsVisible && fPadding && fToText && textCDO;
    if (canTune) {
        oScreenVeh = off(cChild, "Vehicle"); oBox = off(cScreen, "TuningVerticalBox");
        oRowName = off(cRowBase, "NameTextBlock"); oRowSpin = off(cRowBase, "SpinBoxString");
        oSpinText = off(cSpinRow, "TextBlock");
        oBtnName = off(cButtonBase, "NameTextBlock"); oBtnRich = off(cButtonBase, "NameRichTextBlock");
        oBtnText = off(cButtonBase, "ButtonName"); oBtnInner = off(cButtonBase, "MenuButton");
        for (int o : {oScreenVeh, oBox, oRowName, oRowSpin, oSpinText, oBtnName, oBtnRich, oBtnText, oBtnInner}) canTune = canTune && o >= 0;
        auto range = find_sig("48 89 5C 24 08 48 89 74 24 10 57 48 83 EC 20 0F B6 44 24 50 48 8B D9 88 81 ?? ?? ?? ?? F3 0F 11 89");
        auto value = find_sig("40 53 48 83 EC 20 80 B9 ?? ?? ?? ?? 00 48 8B D9 F3 0F 10 81 ?? ?? ?? ?? F3 0F 5D C1 F3 0F 5F 81");
        if (range.size() == 1 && value.size() == 1) {
            spinRange = (decltype(spinRange))range[0]; spinSet = (decltype(spinSet))value[0];
            oSpinValue = get<int>(range[0] + 57);  // the fourth store: value = min
        } else canTune = false;
    }
    static const char* kConstruct[] = {"Construct"};
    if (canTune) canTune = api->watch_functions(kConstruct, 1, onScreenBuilt, nullptr) != 0;
    MT_LOG("Tuning > Brakes & Suspension %s", canTune ? "ready" : "OFF -- a piece of the Tuning screen is missing");
    CloseHandle(CreateThread(nullptr, 0, pump, nullptr, 0, nullptr));
    for (DWORD scanAt = GetTickCount();;) {
        if ((int)(GetTickCount() - scanAt) >= 0) {
            scanAt = GetTickCount() + 2000;  // about two seconds: the vehicles, and the blueprint wheel positions
            std::vector<Ref> v;
            std::map<uint64_t, V> r;
            for (int i = 0, n = api->num_objects(); i < n; i++) {
                uintptr_t o = api->object(i);
                if (!o) continue;
                uintptr_t c = classOf(o);
                if (api->is_a(c, cVeh)) { if (!isCDO(o)) v.push_back({i, o}); continue; }
                if (!api->is_a(c, cWheel)) continue;
                // the blueprint's own wheel: a CDO subobject, or an SCS template "Wheel0_GEN_VARIABLE" on the class
                uintptr_t outer = outerOf(o);
                if (!outer) continue;
                if (isCDO(outer)) {
                    r[(uint64_t)get<int>(classOf(outer) + L().obj_index) << 32 | get<uint32_t>(o + L().obj_name)] = readV(o + oRelLoc);
                } else if (api->is_a(classOf(outer), cClass)) {
                    std::string nm = objName(o);
                    size_t cut = nm.rfind("_GEN_VARIABLE");
                    int base = cut != std::string::npos ? api->name_index(nm.substr(0, cut).c_str()) : 0;
                    if (base > 0) r[(uint64_t)get<int>(outer + L().obj_index) << 32 | (uint32_t)base] = readV(o + oRelLoc);
                }
            }
            std::lock_guard<std::mutex> g(lock);
            found.swap(v); foundRest.swap(r);
        }
        if (disabled) return 0;
        Sleep(50);
    }
}

extern "C" __declspec(dllexport) int PluginInit(const MtApi* a) {
    if (!attach(a)) return 1;
    migrateIni();  // before anything opens the new ini: iniDisabled would create it and the move would skip
    if (iniDisabled(L"suspension_tuning.ini")) return 0;
    CloseHandle(CreateThread(nullptr, 0, run, nullptr, 0, nullptr));
    return 0;
}
