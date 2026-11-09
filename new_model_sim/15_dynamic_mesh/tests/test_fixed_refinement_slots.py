"""All build/QA/process objects below are offline synthetic fixtures only."""
import ast
import concurrent.futures
import copy
import json
import os
from pathlib import Path
import shutil
import sys
import time
import types
import pytest

STAGE=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(STAGE))
import fixed_refinement_bridge as f
import fixed_refinement_construction as worker
import inspection_batch002 as shared
import inspection_batch002_bridge as old
import mcp_bridge as legacy
import mesh_prepare
from fixed_refinement_config import trial_settings,configuration_comparison
from test_definition_probe import sandbox,dump
from test_inspection_batch002 import batch,ready,complete_record_fixture
from test_fixed_refinement_config import native_sphere_fixture
from test_native_stdin_lifetime import FakePipe


def rebind(root):
    slot=f.FixedConstructionSlot(root)
    dump(root/f.MANIFEST,{'schema_version':1,'files':slot._sources()})
    grant=copy.deepcopy(f.POLICY)
    grant.update(authorization_source='OFFLINE fake actual user authorization',recorded_local='offline',execution_policy='Never execute native',source_manifest_sha256=legacy.sha256(root/f.MANIFEST))
    dump(root/f.GRANT,grant)


@pytest.fixture
def fixed(batch,monkeypatch):
    root,_,popen=batch
    monkeypatch.setattr(f,'CONSTRUCTION_RETIRED',False)  # historical fake construction only
    for relative in f.SOURCES:
        path=root/relative;path.parent.mkdir(parents=True,exist_ok=True)
        if not path.exists():path.write_text('# offline fixed source\n',encoding='utf-8')
        if relative.endswith('.in'):path.write_bytes((STAGE/relative).read_bytes())
    user=legacy.read_json(STAGE/f.USER_AUTH);dump(root/f.USER_AUTH,user)
    monkeypatch.setattr(f,'USER_AUTH_SHA',legacy.sha256(root/f.USER_AUTH))
    for domain,row in list(f.BASELINE_MESHES.items()):
        path=root/row['relative'];path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(('OFFLINE baseline '+domain).encode())
        monkeypatch.setitem(f.BASELINE_MESHES,domain,dict(row,size=path.stat().st_size,sha256=legacy.sha256(path)))
    rebind(root)
    monkeypatch.setattr(f.FixedConstructionSlot,'preflight',lambda self:self.authorize())
    monkeypatch.setattr(f.FixedQASlot,'preflight',lambda self:self.authorize())
    monkeypatch.setattr(worker,'environment_clear',lambda:True)
    return root,popen


def reserve_build(root):
    slot=f.FixedConstructionSlot(root);path=slot.safe(slot.ATTEMPT);path.mkdir(parents=True)
    req=slot.make_request(time.time()-30,slot._sources());dump(path/'request.json',req);slot.preserve_sources(req)
    dump(path/'launcher.json',{'job_id':slot.JOB_ID,'pid':os.getpid(),'creation_time':123.5})
    dump(root/'.mesh-inspection-dispatch.lock',{'job_id':slot.JOB_ID,'token':'0'*32})
    return slot,path,req


def native_stats(config):
    stats=native_sphere_fixture(config)
    stats.update(settings=copy.deepcopy(config),domain=config['domain'],mesh_generated=True,nodes=100,elements=200,
        fluid_body_geometry_ids=[7],tet_method_readback='AllTriAllTet',
        readback={'ElementSize':str(config['global_size_m'])+' [m]','MaximumSize':str(config['maximum_size_m'])+' [m]',
            'MinimumSize':str(config['minimum_size_m'])+' [m]','GrowthRate':'1.3','CaptureCurvature':'False','CaptureProximity':'False'},
        local_readback=[{'name':'stage15_'+row['name'],'source_named_selection':row['name'],'size':str(row['size_m'])+' [m]',
            'local_growth_override_applied':False,'global_growth_rate_requested':1.3,'expected_geometry_ids':[i+10],
            'actual_geometry_ids':[i+10],'definition':{'properties':{'ObjectState':'FullyDefined'}}} for i,row in enumerate(config['face_controls'])])
    return stats


def export_fixture(output,configs):
    systems=[];rows=[]
    for index,config in enumerate(configs):
        domain=config['domain'];name='FFF' if index==0 else 'FFF-1'
        mech=output/'stage15_two_domains_files/dp0'/name/'MECH';mech.mkdir(parents=True)
        source=mech/(name+'.msh');source.write_bytes(('OFFLINE new trial mesh '+domain).encode())
        target=output/(domain+'.msh');shutil.copyfile(source,target)
        stats_path=output/(domain+'_meshing_stats.json');dump(stats_path,native_stats(config))
        systems.append({'domain':domain,'system_name':name,'mesh_export':{'source_path':str(source),'path':str(target),'size':target.stat().st_size}})
        rows.append({'domain':domain,'relative':target.relative_to(output.parents[2]).as_posix(),'size':target.stat().st_size,
            'sha256':legacy.sha256(target),'nodes':100,'elements':200,'stats_sha256':legacy.sha256(stats_path),'native_spheres_verified':True})
    dump(output/'workbench_completion.json',{'status':'completed_pending_mesh_inventory','mesh_exports_written':True,'systems':systems})
    dump(output/'workbench_execution.json',{'exit_code':0,'owned_cleanup_verified':True})
    return rows


def completed_build(root):
    slot,path,req=reserve_build(root)
    frozen=worker.probe.validate_frozen(root);output=slot.safe(f.MESH_ATTEMPT)
    output.mkdir(parents=True)
    configs=trial_settings(frozen['contract'],output)
    for config in configs:config['cad_path']=frozen['cad'][config['domain']]['path']
    journal=output/'mesh_stage15.wbjn';journal.write_text('# OFFLINE fixed generated journal\n',encoding='utf-8')
    dump(output/'mesh_plan.json',{'domains':configs,'journal':{'path':str(journal),'sha256':legacy.sha256(journal)}})
    dump(output/'fixed_refinement_configuration.json',configuration_comparison(frozen['contract'],output))
    rows=export_fixture(output,configs)
    dump(output/'workbench_execution.json',{'exit_code':0,'owned_cleanup_verified':True,'solver_launched':False,'automatic_retries':0,
        'total_timeout_seconds':7200,'system_timeout_seconds':3600,'total_deadline_unix':req['deadline_unix'],
        'work_deadline_unix':req['work_deadline_unix'],'journal_sha256':legacy.sha256(journal),'attempt':str(output)})
    final={'job_id':f.BUILD_JOB,'trial_id':f.TRIAL,'operation':slot.OPERATION,'status':'completed',
        'owned_cleanup_verified':True,'inputs_reverified_after_cleanup':True,'sources_reverified_after_cleanup':True,'serial_dispatch_released':True,
        'native_spheres_verified':True,'mesh_exports_written':True,'mesh_generated':True,'solver_launched':False,'flow_iterations':0,
        'mesh_motion_verified':False,'workbench_exit_code':0,'elapsed_seconds':22.,'finished_unix':req['created_unix']+22.,'protected_inputs_before':frozen['protected_inputs'],
        'protected_inputs_after':frozen['protected_inputs'],'meshes':rows,
        **{k:req[k] for k in ('created_unix','deadline_unix','work_deadline_unix')}}
    for filename,key in (('mesh_stage15.wbjn','journal_sha256'),('workbench_completion.json','completion_sha256'),('workbench_execution.json','execution_sha256'),
        ('mesh_plan.json','plan_sha256'),('fixed_refinement_configuration.json','comparison_sha256')):final[key]=legacy.sha256(output/filename)
    dump(path/'probe_result.json',final)
    (root/'.mesh-inspection-dispatch.lock').unlink()  # synthetic completed worker cleanup
    return slot,path,output,final


def test_three_independent_persistent_slots_and_original_paths_preserved(fixed,monkeypatch):
    root,popen=fixed
    slot=f.FixedConstructionSlot(root)
    monkeypatch.setattr(slot,'_inspect_pid',lambda pid:{'pid':pid,'exists':True,'creation_time':123.5})
    assert slot.start(f.START_BUILD)['new_worker_started'] is True
    assert slot.start(f.START_BUILD)['new_worker_started'] is False and slot.status()['remaining_attempts']==0
    request=legacy.read_json(slot.safe(slot.ATTEMPT+'/request.json'))
    assert request['deadline_unix']-request['created_unix']==7200 and request['work_deadline_unix']-request['created_unix']==7170
    for domain in ('background','arm'):
        qa=f.FixedQASlot(domain,root)
        with pytest.raises((legacy.BridgeError,OSError)):qa.start(f.START_QA)
        assert not qa.safe(qa.ATTEMPT).exists()
        assert not (root/'runs/mcp_local_quality/batch_001'/domain).exists()
    assert popen.call_count==1


@pytest.mark.parametrize('token',['NO',old.START,old.FULL_RECORD_START,f.START_QA,f.CANCEL_BUILD])
def test_wrong_build_token_never_consumes(fixed,token):
    root,popen=fixed;slot=f.FixedConstructionSlot(root)
    with pytest.raises(legacy.BridgeError):slot.start(token)
    assert not slot.safe(slot.ATTEMPT).exists();popen.assert_not_called()


@pytest.mark.parametrize('name',['.ansys-task.lock','.mesh-inspection-dispatch.lock','.sizing-dispatch.lock'])
def test_construction_lock_refusal_without_consuming_or_removing(fixed,name):
    root,popen=fixed;slot=f.FixedConstructionSlot(root);dump(root/name,{'owner':'other'});before=(root/name).read_bytes()
    with pytest.raises(legacy.BridgeError):slot.start(f.START_BUILD)
    assert (root/name).read_bytes()==before and not slot.safe(slot.ATTEMPT).exists();popen.assert_not_called()


def test_build_concurrency_once_and_cancel_token(fixed,monkeypatch):
    root,popen=fixed;slot=f.FixedConstructionSlot(root)
    monkeypatch.setattr(slot,'_inspect_pid',lambda pid:{'pid':pid,'exists':True,'creation_time':123.5})
    def start():
        try:return slot.start(f.START_BUILD)
        except (legacy.BridgeError,FileExistsError):return {}
    with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:results=list(pool.map(lambda _:start(),range(4)))
    assert sum(x.get('new_worker_started') is True for x in results)==popen.call_count==1
    with pytest.raises(legacy.BridgeError):slot.cancel(slot.JOB_ID,f.CANCEL_QA)
    assert slot.cancel(slot.JOB_ID,f.CANCEL_BUILD)['status']=='cancellation_requested'


def test_complete_readback_new_export_source_gate_and_QA_once(fixed,monkeypatch):
    root,popen=fixed;build,path,output,final=completed_build(root)
    gate=f.construction_gate(root)
    assert gate['status']=='passed' and [x['domain'] for x in gate['meshes']]==['background','arm']
    for domain in ('background','arm'):
        qa=f.FixedQASlot(domain,root)
        monkeypatch.setattr(qa,'_inspect_pid',lambda pid:{'pid':pid,'exists':True,'creation_time':123.5})
        assert qa.mesh_input()['path']==f.MESH_ATTEMPT+'/'+domain+'.msh'
        assert qa.start(f.START_QA)['new_worker_started'] is True and qa.start(f.START_QA)['new_worker_started'] is False
        request=legacy.read_json(qa.safe(qa.ATTEMPT+'/request.json'))
        assert request['batch_id']==f.TRIAL and request['construction']==gate and request['deadline_unix']-request['created_unix']==300
        assert request['work_deadline_unix']-request['created_unix']==270
        (root/'.mesh-inspection-dispatch.lock').unlink() # synthetic fake worker finished
    assert popen.call_count==2


@pytest.mark.parametrize('fault',['radius','centre','size','sphere_missing','body_scope','global_size','growth','capture','face_size','face_scope','mesh','statistics','journal',
    'plan','comparison','source','archive','grant','user_auth','cleanup','input','deadline','elapsed','solver','old_mesh','execution','completion','native_export','finished','bool_exit'])
def test_gate_corruption_refuses_QA_without_consumption(fixed,fault):
    root,popen=fixed;build,path,output,final=completed_build(root)
    stats_path=output/'background_meshing_stats.json';stats=legacy.read_json(stats_path)
    if fault=='radius':stats['sphere_readback'][-1]['radius']='51 [mm]'
    if fault=='centre':stats['sphere_readback'][-1]['centre'][0]='0 [m]'
    if fault=='size':stats['sphere_readback'][-1]['element_size']='1 [m]'
    if fault=='sphere_missing':stats['sphere_readback'].pop()
    if fault=='body_scope':stats['fluid_body_geometry_ids']=[999]
    if fault=='global_size':stats['readback']['ElementSize']='100 [mm]'
    if fault=='growth':stats['readback']['GrowthRate']='1.2'
    if fault=='capture':stats['readback']['CaptureProximity']='True'
    if fault=='face_size':stats['local_readback'][0]['size']='4 [mm]'
    if fault=='face_scope':stats['local_readback'][0]['actual_geometry_ids']=[999]
    if fault in ('radius','centre','size','sphere_missing','body_scope','global_size','growth','capture','face_size','face_scope'):
        dump(stats_path,stats);final['meshes'][0]['stats_sha256']=legacy.sha256(stats_path)
    if fault=='native_export':
        source=Path(legacy.read_json(output/'workbench_completion.json')['systems'][0]['mesh_export']['source_path'])
        data=source.read_bytes();source.write_bytes(b'X'+data[1:])  # same size, wrong content
    if fault=='finished':final['finished_unix']=final['deadline_unix']+1
    if fault=='bool_exit':final['workbench_exit_code']=False
    if fault=='mesh':(output/'background.msh').write_bytes(b'changed')
    if fault=='statistics':dump(stats_path,{'native_spheres_verified':True})
    if fault=='journal':(output/'mesh_stage15.wbjn').write_bytes(b'changed')
    if fault=='plan':dump(output/'mesh_plan.json',{'domains':[]})
    if fault=='comparison':dump(output/'fixed_refinement_configuration.json',{'native_readback_verified':True})
    if fault=='source':(root/'fixed_refinement_config.py').write_bytes(b'changed')
    if fault=='archive':(path/'source_snapshot/fixed_refinement_config.py').write_bytes(b'changed')
    if fault=='grant':dump(root/f.GRANT,{'construction_attempts':2})
    if fault=='user_auth':dump(root/f.USER_AUTH,{'automatic_retries':1})
    if fault=='cleanup':final['owned_cleanup_verified']=False
    if fault=='input':(root/'contract.json').write_bytes(b'changed')
    if fault=='deadline':final['deadline_unix']+=1
    if fault=='elapsed':final['elapsed_seconds']=7201
    if fault=='solver':final['solver_launched']=True
    if fault=='old_mesh':final['meshes'][0]['relative']=f.BASELINE_MESHES['background']['relative']
    if fault=='execution':dump(output/'workbench_execution.json',{'exit_code':0,'owned_cleanup_verified':False})
    if fault=='completion':dump(output/'workbench_completion.json',{'mesh_exports_written':True,'systems':[]})
    dump(path/'probe_result.json',final)
    qa=f.FixedQASlot('background',root)
    with pytest.raises((legacy.BridgeError,worker.RuntimeGateError,OSError,ValueError,KeyError)):qa.start(f.START_QA)
    assert not qa.safe(qa.ATTEMPT).exists();popen.assert_not_called()


def test_accepted_build_blocks_other_dispatch_before_native_task_lock(fixed,monkeypatch):
    root,popen=fixed;slot=f.FixedConstructionSlot(root)
    monkeypatch.setattr(slot,'_inspect_pid',lambda pid:{'pid':pid,'exists':True,'creation_time':123.5})
    assert slot.start(f.START_BUILD)['new_worker_started'] is True
    assert not (root/'.ansys-task.lock').exists() and (root/'.mesh-inspection-dispatch.lock').exists()
    from local_quality_diagnostic import LocalQualitySlot,START
    local=LocalQualitySlot('background',root)
    monkeypatch.setattr(local,'authorize',lambda:{})
    monkeypatch.setattr(local,'preflight',lambda:{'status':'passed'})
    with pytest.raises(legacy.BridgeError):local.start(START)
    assert not local.safe(local.ATTEMPT).exists() and popen.call_count==1


def test_missing_stats_during_build_is_failed_and_keeps_originals(fixed,monkeypatch):
    root,popen=fixed;slot,path,req=reserve_build(root)
    before={k:(root/v['relative']).read_bytes() for k,v in f.BASELINE_MESHES.items()}
    monkeypatch.setattr(worker,'validate_launcher',lambda *a:{'pid':os.getpid(),'creation_time':123.5})
    calls=[]
    def fake(attempt,**kwargs):
        plan=legacy.read_json(attempt/'mesh_plan.json');export_fixture(attempt,plan['domains'])
        stats=legacy.read_json(attempt/'arm_meshing_stats.json');stats.pop('sphere_readback');dump(attempt/'arm_meshing_stats.json',stats)
        calls.append(True);return 0
    monkeypatch.setattr(worker.run_mesh_job,'run',fake)
    result=worker.run_build(path,stage_root=root)
    assert result['status']=='failed' and not result['native_spheres_verified'] and result['owned_cleanup_verified']
    assert len(calls)==1 and all((root/f.BASELINE_MESHES[k]['relative']).read_bytes()==v for k,v in before.items())
    assert not f.FixedQASlot('arm',root).safe(f.FixedQASlot('arm',root).ATTEMPT).exists()
    popen.assert_not_called()


def test_build_worker_calls_only_existing_bounded_workbench_runner_and_actual_readback(fixed,monkeypatch):
    root,popen=fixed;slot,path,req=reserve_build(root)
    output=slot.safe(f.MESH_ATTEMPT)
    monkeypatch.setattr(worker,'validate_launcher',lambda *a:{'pid':os.getpid(),'creation_time':123.5})
    called=[]
    def native_build(attempt,**kwargs):
        assert kwargs['total_deadline_unix']==req['deadline_unix'] and not kwargs['cancel_check']()
        plan=legacy.read_json(attempt/'mesh_plan.json');export_fixture(attempt,plan['domains'])
        dump(attempt/'workbench_execution.json',{'exit_code':0,'owned_cleanup_verified':True,'solver_launched':False,'automatic_retries':0,
            'total_timeout_seconds':7200,'system_timeout_seconds':3600,'total_deadline_unix':req['deadline_unix'],
            'work_deadline_unix':req['work_deadline_unix'],'journal_sha256':legacy.sha256(attempt/'mesh_stage15.wbjn'),'attempt':str(attempt)})
        called.append(True);return 0
    monkeypatch.setattr(worker.run_mesh_job,'run',native_build)
    report=worker.run_build(path,stage_root=root)
    assert report['status']=='completed' and report['native_spheres_verified'] is True and report['mesh_quality_verified'] is False
    assert report['solver_launched'] is False and report['flow_iterations']==0 and len(called)==1
    assert report['owned_cleanup_verified'] and report['inputs_reverified_after_cleanup'] and report['sources_reverified_after_cleanup'] and report['serial_dispatch_released']
    assert not (root/'.mesh-inspection-dispatch.lock').exists()
    assert f.construction_gate(root)['status']=='passed'
    popen.assert_not_called()


def test_new_QA_native_read_only_reads_new_mesh_and_still_fails_low_quality(fixed,monkeypatch):
    root,popen=fixed;build,buildpath,output,final=completed_build(root)
    qa=f.FixedQASlot('background',root);path=qa.safe(qa.ATTEMPT);path.mkdir(parents=True)
    req=qa.make_request(time.time(),qa._sources());dump(path/'request.json',req);qa.preserve_sources(req)
    parent=os.getppid();dump(path/'launcher.json',{'job_id':qa.JOB_ID,'pid':parent,'creation_time':123.5});dump(root/'.ansys-task.lock',{'pid':parent,'attempt':str(path)})
    monkeypatch.setenv('REMOTING_SERVER_ADDRESS','127.0.0.1')
    # Gate was fully checked above; repeat calls use same immutable synthetic gate.
    monkeypatch.setattr(f.FixedQASlot,'connection_gate',lambda self:{'meshes':final['meshes']})
    exe=root/'OFFLINE_NEVER_EXECUTE.exe';exe.write_bytes(b'offline fixture')
    monkeypatch.setattr(shared,'validate_frozen',lambda *a:{'contract':{'software':{'fluent':str(exe),'ansys_root':str(root)}}})
    monkeypatch.setattr(shared.psutil,'Process',lambda pid=None:types.SimpleNamespace(pid=os.getpid() if pid is None else pid,
        create_time=lambda:123.5,ppid=lambda:os.getpid(),exe=lambda:str(exe)))
    events=[];pipe=FakePipe(events);process=types.SimpleNamespace(pid=501,stdin=pipe,poll=lambda:None,wait=lambda **k:0)
    launches=[]
    def launch(*args,**kwargs):launches.append(kwargs);return process
    monkeypatch.setattr(shared.subprocess,'Popen',launch)
    reads=[]
    def start_transcript(value):Path(value).write_text('Minimum Orthogonal Quality = 0.0007\nminimum volume (m3): 1e-9\nmaximum volume (m3): 1\n',encoding='utf-8')
    class Walls:
        def get_object_names(self):return old.MESHES['background']['walls']
        def __getitem__(self,name):return types.SimpleNamespace(adjacent_cell_zone=lambda:'fluid_background')
    solver=types.SimpleNamespace(file=types.SimpleNamespace(read_mesh=lambda **k:reads.append(k['file_name'])),
        tui=types.SimpleNamespace(file=types.SimpleNamespace(start_transcript=start_transcript,stop_transcript=lambda:None),
            mesh=types.SimpleNamespace(quality=lambda:None,check=lambda:None,modify_zones=types.SimpleNamespace(list_zones=lambda:None))),
        setup=types.SimpleNamespace(boundary_conditions=types.SimpleNamespace(wall=Walls()),cell_zone_conditions=types.SimpleNamespace(fluid=types.SimpleNamespace(get_object_names=lambda:['fluid_background']))),exit=lambda:None)
    connects=[]
    def connect(*a,**k):connects.append(k);a[3].update(ready(),cleanup_on_exit=True,constructor_watchdog_process_started=False);return solver
    monkeypatch.setattr(shared,'connect_ready',connect)
    # Internal new binding matcher still runs; it compares the fake immutable gate above.
    req['construction']=qa.connection_gate();dump(path/'request.json',req)
    result=shared.native_collect(path,stage_root=root,fixed_refinement=True)
    assert len(launches)==len(connects)==1 and reads==[str(output/'background.msh')]
    assert result['provenance']==f.PROVENANCE and result['inspection']['inspection_gate_passed'] is False
    assert result['normal_exit']['native_returncode']==0 and pipe.closed and result['errors']=={}


def test_fixed_surface_no_parameters_and_original_local_slots_unchanged():
    source=(STAGE/'stage15_mcp.py').read_text(encoding='utf-8');tools={x.name:x for x in ast.parse(source).body if isinstance(x,ast.AsyncFunctionDef)}
    assert len(tools)==26 and [a.arg for a in tools['stage15_start_fixed_refinement_construction'].args.args]==['confirmation']
    assert [a.arg for a in tools['stage15_start_fixed_refinement_QA'].args.args]==['domain','confirmation']
    assert '--QA-preflight' in (STAGE/'fixed_refinement_construction.py').read_text(encoding='utf-8')
    protocol=(STAGE/'check_mcp_connection.py').read_text(encoding='utf-8')
    assert f.START_BUILD not in protocol and f.START_QA not in protocol
    assert '("stage15_start_fixed_refinement_construction", {"confirmation": "NO"})' in protocol
    assert f.POLICY['additional_local_diagnostics']==0 and f.POLICY['additional_connection_probes']==0
