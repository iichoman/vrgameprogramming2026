"""Candidate process classification; all non-wheel motion requires interface review."""
WHEELS = {
    "Wheel_FL": set(range(2000424, 2000428)),
    "Wheel_FR": set(range(2000491, 2000495)),
    "Wheel_RL": set(range(2000487, 2000491)),
    "Wheel_RR": set(range(2000483, 2000487)),
}
GLASS_BODY = {2000290:"Windshield",2000647:"Windshield",2000648:"Windshield",2000291:"RearGlass",2000977:"RearGlass",2000978:"RearGlass"}
STEERING = {2000675, *range(2000700,2000712), *range(2000715,2000718)}
COCKPIT = {*range(2000676,2000700),*range(2000712,2000715),*range(2000718,2000744)}
BODY_INTERFACES = {2000003,2000006,2000008,2000071,2000072,2000075,2000077,2000094,2000104,2000516,2000578,2000649,2000650,2000651,2000652,2001057,2001058,*range(2001111,2001115),*range(2005000,2005004)}
BODY_INTERFACES.update({2000103,2000127,2000228,2000229,2000230,2000231,2000232,2000265,2000266,2000267})
SUBFRAME_TYPOS={2000324,2000332,2000334,2000378,2000387,2000388,2000392}
RUNNING_GEAR_TYPOS={2000403,2000416,2000418,2000526,2000529,2000531,2000541,2000565,2000566,2000569,2000570,2000571}

def classify(p):
    pid,n=p['part_id'],p['name'].lower()
    lo,hi=p['bounds_m']
    x,y,z=p['center_m']
    side='L' if y>=0 else 'R'
    for module,ids in WHEELS.items():
        if pid in ids: return module,'verified','Exact wheel PID set; corner confirmed by geometry.'
    if pid in BODY_INTERFACES:
        return 'Body_InterfaceReview','review','Body-side support or FE attachment; keep fixed until interface is reviewed.'
    if pid in GLASS_BODY:
        return GLASS_BODY[pid],'candidate','Exact glass PID; laminate visibility preserved.'
    if 'door' in n:
        front=any(t in n for t in ('frontdoor','doorfront'))
        rear=any(t in n for t in ('reardoor','doorrear','doorear'))
        if not (front or rear): rear=x < -2.6
        return 'Door_'+('R' if rear else 'F')+side,'candidate','Door-family token (not generic front/rear word); geometry resolves side and unlabeled speaker.'
    if 'hinge' in n:
        return 'Body_InterfaceReview','review','Door/closure hinge halves must be assigned to body or moving leaf explicitly.'
    if 'seat_att' in n:
        return 'Body_InterfaceReview','review','FE attachment surrogate is not a confirmed moving seat part.'
    if pid in {2000992,2001028}: return 'Seat_Rear','candidate','Explicit correction for searrear misspelling.'
    if 'seat' in n or 'headrest' in n or 'head_rest' in n:
        rear=('rear' in n and not ('front' in n or 'driver' in n or 'drvr' in n or 'drver' in n)) or 'bench' in n
        if x < -2.8: rear=True
        return ('Seat_Rear' if rear else 'Seat_Front_'+side),'candidate','Seat names and longitudinal location; rails/feet require interface validation.'
    if pid in STEERING:
        is_wheel=pid in {2000675,2000701,2000702,2000703,2000704,2000710}
        return ('SteeringWheel' if is_wheel else 'SteeringColumn'),'candidate','Explicit cockpit steering PID list; excludes steering rack; bracket boundary needs review.'
    if 'ipbeam' in n:
        return 'CockpitFrame','candidate','Instrument-panel structural beam; separate from steering rack and body shell.'
    if pid in COCKPIT or any(t in n for t in ('dash','instrument','glove','cockpit')):
        return 'Cockpit','candidate','Cockpit/dashboard assembly; exact installation boundary requires review.'
    if 'console' in n:
        return 'CenterConsole','candidate','Center-console names.'
    if 'hood' in n:
        return ('Body_InterfaceReview','review','Latch/mount boundary requires review.') if any(t in n for t in ('latch','hinge')) else ('Hood','candidate','Hood named panel/support.')
    if 'trunk' in n:
        if pid in {2000293,2000294}: return 'TrunkLid','candidate','Exact inner and outer trunk lid panels.'
        return 'Body_Review','review','Trunk floor/side/crossmember is fixed body, not removable trunk lid.'
    if 'bumper' in n:
        return ('Bumper_Front' if x>-2.5 else 'Bumper_Rear'),'candidate','Position distinguishes front/rear; NULL duplicates retained and flagged.'
    if 'headlight' in n or 'taillight' in n:
        return ('Lamp_Front_' if 'headlight' in n else 'Lamp_Rear_')+side,'candidate','Lamp names and geometry side.'
    if any(t in n for t in ('radiatorsupport','radiatorcrossmember','struthousing','subframebodybrkt','enginemtbrkt','wheelhousing','wheelwell')):
        return 'Body_InterfaceReview','review','Support/housing may belong to welded body; never infer moving module from keyword.'
    if 'enginemount' in n or 'engnemount' in n or 'transmissionsupport' in n:
        return 'PowertrainMounts_Review','review','Mount rubber/bracket attachment boundary not inferred from name.'
    if any(t in n for t in ('engine','transmission','waterpump','alternator','airfilter','intakemanifold')):
        return ('Transmission' if 'transmission' in n else 'Engine'),'candidate','ICE powertrain geometry; not a Tesla EV powertrain.'
    if any(t in n for t in ('radiator','coolant')):
        return 'CoolingModule','candidate','Cooling hardware; body supports excluded above.'
    if 'battery' in n:
        return 'AuxBattery','candidate','Camry auxiliary battery, not an EV traction battery pack.'
    if any(t in n for t in ('exhaust','muffler')):
        return 'ExhaustSystem','candidate','ICE exhaust components.'
    if 'fuel' in n or 'gastank' in n:
        return 'FuelSystem','candidate','ICE fuel components.'
    if 2000604 <= pid <= 2000608:
        return 'BrakeBooster','candidate','Explicit correction for breakbooster spelling; not a wheel component.'
    if pid in SUBFRAME_TYPOS:
        return 'Subframe'+('_Front' if x>-2.5 else '_Rear'),'candidate','Explicit source subframe spelling correction.'
    if pid in RUNNING_GEAR_TYPOS:
        return 'RunningGear'+('_Front' if x>-2.5 else '_Rear'),'candidate','Explicit rotor/disk/suspension source-name correction; excluded from tire/rim.'
    if any(t in n for t in ('steeringrack','subframe','susp','strut','aarm','axle','spindle','hub','shock','spring','antisway')):
        kind='Subframe' if 'subframe' in n else 'RunningGear'
        return kind+('_Front' if x>-2.5 else '_Rear'),'candidate','Running gear by location; supports/mounts still require interface review.'
    if 'brake' in n or 'pedal' in n or 'accelerator' in n:
        return 'PedalBrake_Review','review','Name may denote cabin pedal mechanism rather than wheel brake.'
    if any(t in n for t in ('carpet','headlin','trim','airbag','duct','vent','belt')):
        return 'InteriorFixed_Review','review','Fixed interior/mounts pending assignment; no automatic motion.'
    return 'Body_Review','review','Unclassified retained geometry; keep fixed; review rather than silently omit.'


def classify_part(pid, name, bounds):
    p={"part_id":int(pid),"name":name,"bounds_m":bounds,"center_m":[(bounds[0][i]+bounds[1][i])/2 for i in range(3)]}
    return classify(p)
