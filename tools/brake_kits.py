"""Big brake kits: Brake Pads parts for the Suspension Visuals mod.

Since 2026-11-09 (the user's call) a kit is a Brake Pads part, one row per disc size (C0001_BBK<mm>): a bigger disc
soaks and sheds more heat, so the kit's BrakePad stats are what change -- heating, cooling, fade temperature, wear --
open to every vehicle class (vanilla pads are gated per class). Colour comes from the Paint screen, not the row.
The 18 earlier BrakePower rows (C0001_BBK<mm>_<Colour>) stay in the table hidden: row names are save keys.

    python BrakeKits/run.py          # after Rebase/channels.py and NewEngines/synth.py (it strips the kits out of
                                     # MoreTuning's table, where they used to live)

Suspension Visuals is its own mod (the user's call, 2026-11-09): qxZap_SuspensionVisuals_P ships the kits in the
framework's reserved VehicleParts Slot01 (free on this machine, Arini checked; a slot needs no framework rebuild)
and the meshes under /Game/SuspensionVisuals/. No composite copy: the framework's lists the slot.
    python BrakeKits/run.py demo

Each size is a stronger BrakePowerMultiplier (vanilla: BrakePower_130/150/200 = x1.3/1.5/2.0 for
500/1000/5000), and each colour a row of its own. The row name is the metadata the SuspensionVisuals
plugin reads (C0001_BBK<disc mm>_<Colour>): disc size and caliper colour. Without the plugin they are
plain brake upgrades. Rerunnable: it replaces its own rows. Row names are save keys -- never rename.

Each kit also carries four paint slots (ColorSlots, the map wheels use for their rims), so the game's own Paint
screen lists Brake Caliper, Brake Disc, Suspension and Dampers once a kit is fitted. The plugin reads the colours
back from the vehicle's customisation by slot name (QX_*); no mesh has those slot names, so nothing else repaints.
"""
import copy
import importlib.util
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.join(HERE, "..")
sys.path.insert(0, os.path.join(REPO, "Rebase"))
from pipeline import MOD, VANILLA, WORK, name_property, register_names, to_json  # noqa: E402
from rebase import fnames_in  # noqa: E402
sys.path.insert(0, os.path.join(REPO, "RemoveReq"))
from run import strip_row  # noqa: E402

_spec = importlib.util.spec_from_file_location("dtframework", os.path.join(REPO, "DataTableFramework", "run.py"))
framework = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(framework)

TABLE_PKG = "/Game/ReservedRegistration/Slots/Slot01/DT_VehicleParts"
DEST = os.path.join(REPO, "qxZap_SuspensionVisuals_P", "MotorTown", "Content")
OLD_PKG = "/Game/DataAsset/VehicleParts/VehicleParts_MoreTuning"  # where the kits were until 2026-11-09
OLD = os.path.join(MOD, "DataAsset", "VehicleParts", "VehicleParts_MoreTuning.uasset")
DONOR_TABLE = os.path.join(VANILLA, "DataAsset", "VehicleParts", "BrakePower.uasset")
DONOR = "BrakePower_200"
PAD_TABLE = os.path.join(VANILLA, "DataAsset", "VehicleParts", "BrakePads.uasset")
PAD_DONOR = "BrakePad_Small_02"
# disc mm: heating, cooling, fade temperature (C), wear, cost, kg -- vanilla's best pad is 0.95 / 1.1 / 450 / 0.9
PADS = {320: (0.85, 1.3, 550.0, 0.8, 3000, 4.0), 360: (0.75, 1.55, 625.0, 0.7, 6000, 6.0), 400: (0.65, 1.8, 700.0, 0.6, 10000, 8.0)}
PREFIX = "C0001_BBK"
SIZES = [(320, 1.7, 3000, 4.0), (360, 2.1, 6000, 6.0), (400, 2.5, 10000, 8.0)]  # disc mm, multiplier, cost, kg
COLOURS = ["Red", "Yellow", "Blue", "Black", "Silver", "Gold"]  # the plugin's caliper palette, by name
RGB = {"Red": "140, 5, 3", "Yellow": "217, 153, 5", "Blue": "5, 26, 140", "Black": "5, 5, 5", "Silver": "179, 181, 186",
       "Gold": "191, 133, 31"}  # the plugin's COLOURS, 0-255
PAINT = [("QX_BrakeCaliper", "Brake Caliper"), ("QX_BrakeDisc", "Brake Disc"), ("QX_Suspension", "Suspension"),
         ("QX_Damper", "Dampers")]  # slot names are the plugin's keys: never rename
PAINT_DEFAULT = {"QX_BrakeDisc": "15, 15, 17", "QX_Suspension": "77, 77, 82", "QX_Damper": "200, 150, 20"}
SLOT_DONOR = ("Wheels.uasset", "Bulko")  # a vanilla row with one ColorSlots entry: the template


def kits():
    return [(f"{PREFIX}{mm}_{c}", f"Big Brake Kit {mm} mm ({c})", k, cost, kg)
            for mm, k, cost, kg in SIZES for c in COLOURS]


def paint_slots(template, colour):
    """ColorSlots entries [key, value] for PAINT, cloned from a vanilla wheel's one entry."""
    out = []
    for slot, label in PAINT:
        k, v = copy.deepcopy(template)
        k["Value"] = slot
        for f in v["Value"]:
            if f["Name"] == "DisplayName":
                f.update({x: y for x, y in name_property(label).items() if x != "Name"})
            elif f["Name"] == "DefaultColor":
                f["Value"][0]["Value"] = RGB[colour] if slot == "QX_BrakeCaliper" else PAINT_DEFAULT[slot]
        out.append([k, v])
    return out


def make_row(donor, name, label, mult, cost, kg, template=None):
    row = copy.deepcopy(donor)
    row["Name"] = name
    for i, p in enumerate(row["Value"]):
        if p["Name"] == "Name":
            row["Value"][i] = name_property(label)
        elif p["Name"] == "Cost":
            p["Value"] = cost
        elif p["Name"] == "MassKg":
            p["Value"] = kg
        elif p["Name"] == "BrakePower":
            [m for m in p["Value"] if m["Name"] == "BrakePowerMultiplier"][0]["Value"] = mult
        elif p["Name"] == "ColorSlots" and template:
            p["Value"] = paint_slots(template, name.rsplit("_", 1)[1])
    return row


def make_pad(donor, mm):
    heat, cool, fade, wear, cost, kg = PADS[mm]
    row = copy.deepcopy(donor)
    row["Name"] = f"{PREFIX}{mm}"
    strip_row(row)
    for i, p in enumerate(row["Value"]):
        if p["Name"] == "Name":
            row["Value"][i] = name_property(f"Big Brake Kit {mm} mm")
        elif p["Name"] == "Cost":
            p["Value"] = cost
        elif p["Name"] == "MassKg":
            p["Value"] = kg
        elif p["Name"] == "BrakePad":
            for f in p["Value"]:
                f["Value"] = {"HeatingMultiplier": heat, "CoolingMultiplier": cool, "FadeTemperature": fade, "WearMultiplier": wear}[f["Name"]]
    return row


def hide(row):
    for p in row["Value"]:
        if p["Name"] == "bIsHidden":
            p["Value"] = True
    return row


def main():
    work = os.path.join(WORK, "brakekits")
    os.makedirs(work, exist_ok=True)
    donor = {r["Name"]: r for r in framework.load(to_json(DONOR_TABLE, os.path.join(work, "donor.json")))["Exports"][0]["Table"]["Data"]}[DONOR]
    wheels = framework.load(to_json(os.path.join(VANILLA, "DataAsset", "VehicleParts", SLOT_DONOR[0]), os.path.join(work, "wheels.json")))
    wrow = {r["Name"]: r for r in wheels["Exports"][0]["Table"]["Data"]}[SLOT_DONOR[1]]
    template = [p for p in wrow["Value"] if p["Name"] == "ColorSlots"][0]["Value"][0]
    donor_table = framework.load(to_json(DONOR_TABLE, os.path.join(work, "donor.json")))
    pads = {r["Name"]: r for r in framework.load(to_json(PAD_TABLE, os.path.join(work, "pads.json")))["Exports"][0]["Table"]["Data"]}
    new = [hide(make_row(donor, *k)) for k in kits()] + [make_pad(pads[PAD_DONOR], mm) for mm in PADS]
    ours = {r["Name"] for r in new}
    table = framework.make_table(donor_table, TABLE_PKG, new)
    register_names(table, [x for r in new for x in fnames_in(r)] + list(ours))
    check = framework.write_table(table, TABLE_PKG, DEST)
    after = {r["Name"] for r in framework.load(check)["Exports"][0]["Table"]["Data"]}
    if after != ours:
        sys.exit(f"FAIL: slot table has {len(after)} rows, wanted the {len(ours)} kits")
    import subprocess
    subprocess.run([sys.executable, os.path.join(REPO, "Rebase", "verify.py"), check, DEST, MOD, VANILLA], check=True)
    print(f"  Slot01 DT_VehicleParts: {len(PADS)} brake kits (Brake Pads) + {len(ours) - len(PADS)} hidden old rows -> qxZap_SuspensionVisuals_P")

    old = framework.load(to_json(OLD, os.path.join(work, "moretuning.before.json")))
    rows = old["Exports"][0]["Table"]["Data"]
    before = {r["Name"] for r in rows}
    rows[:] = [r for r in rows if not r["Name"].startswith(PREFIX)]
    if len(rows) < len(before):
        check = framework.write_table(old, OLD_PKG, MOD)
        left = {r["Name"] for r in framework.load(check)["Exports"][0]["Table"]["Data"]}
        if left != before - ours:
            sys.exit(f"FAIL: VehicleParts_MoreTuning lost more than the kits: {sorted(before - ours - left)[:10]}")
        subprocess.run([sys.executable, os.path.join(REPO, "Rebase", "verify.py"), check, MOD, VANILLA], check=True)
    print(f"  VehicleParts_MoreTuning: {len(before)} -> {len(rows)} rows (kits moved out)")

if __name__ == "__main__":
    if sys.argv[1:] == ["demo"]:
        k = kits()
        assert len(k) == 18 and len({x[0] for x in k}) == 18 and k[0][0] == "C0001_BBK320_Red"
        d = {"Name": "x", "Value": [{"Name": "Name"}, {"Name": "Cost", "Value": 1}, {"Name": "MassKg", "Value": 1.0},
                                    {"Name": "BrakePower", "Value": [{"Name": "BrakePowerMultiplier", "Value": 2.0}]}]}
        r = make_row(d, *k[0])
        assert r["Value"][0]["CultureInvariantString"] == "Big Brake Kit 320 mm (Red)" and r["Value"][3]["Value"][0]["Value"] == 1.7
        assert d["Value"][3]["Value"][0]["Value"] == 2.0  # donor untouched
        t = [{"Value": "Wheel_01"}, {"Value": [{"Name": "DisplayName", "Value": "k"}, {"Name": "DefaultColor", "Value": [{"Value": "1, 1, 1"}]}]}]
        d["Value"].append({"Name": "ColorSlots", "Value": []})
        sl = make_row(d, "C0001_BBK400_Gold", "x", 2.5, 1, 1.0, template=t)["Value"][-1]["Value"]
        assert [e[0]["Value"] for e in sl] == [x for x, _ in PAINT] and sl[0][1]["Value"][1]["Value"][0]["Value"] == RGB["Gold"]
        assert sl[1][1]["Value"][0]["CultureInvariantString"] == "Brake Disc" and t[0]["Value"] == "Wheel_01"
        print("demo ok")
    else:
        main()
