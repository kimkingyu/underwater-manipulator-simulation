"""No real launch: absent-psutil host dispatch and actual-old/current-QA binding."""
import copy
import json
from pathlib import Path
import sys
import types
import subprocess
import pytest

STAGE=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(STAGE))
import fixed_refinement_bridge as f
import fixed_refinement_history as h
import fixed_refinement_construction as worker
import mcp_bridge as legacy
from test_definition_probe import sandbox,dump
from test_inspection_batch002 import batch
from test_fixed_refinement_slots import fixed,completed_build,rebind


@pytest.fixture
def dual(fixed,monkeypatch):
    root,popen=fixed
    slot,path,output,final=completed_build(root)
    req=legacy.read_json(path/'request.json')
    oldrows=[x for x in req['sources'] if Path(x['path']).name!='fixed_refinement_history.py']
    assert len(oldrows)==29
    dump(root/h.OLD_MANIFEST,{'schema_version':1,'files':oldrows})
    oldgrant=legacy.read_json(STAGE/h.OLD_GRANT)
    oldgrant.update(source_manifest_sha256=legacy.sha256(root/h.OLD_MANIFEST))
    dump(root/h.OLD_GRANT,oldgrant)
    req.update(sources=oldrows,authorization={'path':str(root/h.OLD_GRANT),'sha256':legacy.sha256(root/h.OLD_GRANT)})
    dump(path/'request.json',req)
    archive=legacy.read_json(path/'source_snapshot/archive_manifest.json')
    archive['files']=[x for x in archive['files'] if Path(x['original_path']).name!='fixed_refinement_history.py']
    dump(path/'source_snapshot/archive_manifest.json',archive)
    bindings=[]
    for rel in (f.USER_AUTH,h.OLD_GRANT,h.OLD_MANIFEST):
        target='execution_binding/'+Path(rel).name
        dump(path/'source_snapshot'/target,legacy.read_json(root/rel))
        bindings.append({'original':rel,'archive_relative':target,'sha256':legacy.sha256(root/rel)})
    dump(path/'source_snapshot/execution_binding.json',{'job_id':slot.JOB_ID,'files':bindings})
    dump(path/'worker_claim.json',{'job_id':slot.JOB_ID,'pid':123,'creation_time':123.5})
    old=h.HistoricalConstructionSlot(root)
    rows=[{'path':rel,'size':old.safe(rel).stat().st_size,'sha256':legacy.sha256(old.safe(rel))} for rel in h.historical_paths(old)]
    history={'job_id':slot.JOB_ID,'old_authorization_relative':h.OLD_GRANT,'old_authorization_sha256':legacy.sha256(root/h.OLD_GRANT),
        'old_source_manifest_relative':h.OLD_MANIFEST,'old_source_manifest_sha256':legacy.sha256(root/h.OLD_MANIFEST),'old_source_count':29,
        'additional_construction_attempts':0,'additional_QA_attempts':0,'files':rows}
    dump(root/h.HISTORY,history)
    monkeypatch.setattr(h,'HISTORY_SHA',legacy.sha256(root/h.HISTORY))
    monkeypatch.setitem(f.POLICY,'historical_binding_sha256',h.HISTORY_SHA)
    monkeypatch.setattr(f,'CONSTRUCTION_RETIRED',True)
    monkeypatch.setattr(f.FixedConstructionSlot,'_inspect_pid',lambda self,pid:{'pid':pid,'exists':False})
    rebind(root)
    # New current QA code intentionally differs; historical request stays original.
    changed=root/'fixed_refinement_bridge.py';changed.write_bytes(changed.read_bytes()+b'# corrected QA code\n')
    rebind(root)
    return root,path,output,popen


def test_actual_old_build_and_corrected_QA_sources_are_distinct_and_both_required(dual):
    root,path,output,popen=dual
    before=(path/'probe_result.json').read_bytes()
    gate=f.construction_gate(root)
    assert gate['status']=='passed'
    assert gate['source_manifest_sha256']==legacy.sha256(root/h.OLD_MANIFEST)
    assert gate['grant_sha256']==legacy.sha256(root/h.OLD_GRANT)
    assert gate['new_QA_source_manifest_sha256']==legacy.sha256(root/f.MANIFEST)
    assert gate['historical_binding_sha256']==h.HISTORY_SHA
    assert len(h.HistoricalConstructionSlot(root)._sources())==29 and len(f.FixedQASlot('arm',root)._sources())==30
    assert (path/'probe_result.json').read_bytes()==before
    assert f.FixedQASlot('arm',root).make_request(1.,f.FixedQASlot('arm',root)._sources())['construction']==gate
    popen.assert_not_called()


@pytest.mark.parametrize('target',['oldgrant','oldmanifest','oldsourcearchive','request','history','newsource','newgrant','newmanifest','sphere','execution','mesh'])
def test_either_version_or_runtime_mutation_rejects_without_QA_consumption(dual,target):
    root,path,output,popen=dual
    p={'oldgrant':root/h.OLD_GRANT,'oldmanifest':root/h.OLD_MANIFEST,'oldsourcearchive':path/'source_snapshot/stage15_mcp.py',
       'request':path/'request.json','history':root/h.HISTORY,'newsource':root/'fixed_refinement_history.py','newgrant':root/f.GRANT,
       'newmanifest':root/f.MANIFEST,'sphere':output/'arm_meshing_stats.json','execution':output/'workbench_execution.json','mesh':output/'arm.msh'}[target]
    p.write_bytes(p.read_bytes()+b'changed')
    for name in ('background','arm'):
        qa=f.FixedQASlot(name,root)
        with pytest.raises((legacy.BridgeError,worker.RuntimeGateError,ValueError,OSError)):qa.start(f.START_QA)
        assert not qa.safe(qa.ATTEMPT).exists()
    popen.assert_not_called()


def test_missing_psutil_host_dispatch_uses_only_fixed_official_readonly_script(dual,monkeypatch):
    root,path,output,popen=dual
    valid=f.construction_gate(root)
    monkeypatch.setattr(h,'STAGE',root)
    monkeypatch.setattr(h,'official_process_reader_available',lambda:False)
    calls=[]
    def run(command,**kwargs):
        calls.append((command,kwargs))
        assert command==['offline-python','-B',str(root/'fixed_refinement_history.py'),'--verify-completed-construction']
        assert kwargs['stdin']==subprocess.DEVNULL and kwargs['shell'] is False and kwargs['timeout']==60
        assert kwargs['cwd']==str(root)
        assert not any(x in command for x in ('--run','--native','--QA-preflight'))
        return types.SimpleNamespace(returncode=0,stdout=json.dumps(valid).encode())
    monkeypatch.setattr(h.subprocess,'run',run)
    result=f.construction_gate(root)
    assert result==valid and len(calls)==1
    assert not f.FixedQASlot('background',root).safe(f.FixedQASlot('background',root).ATTEMPT).exists()
    popen.assert_not_called()


@pytest.mark.parametrize('fault',['returncode','oversize','invalidjson','wrongjob','wrongnewsource','wrongoldbinding','timeout'])
def test_host_dispatch_does_not_retry_or_accept_wrong_bounded_evidence(dual,monkeypatch,fault):
    root,path,output,popen=dual
    valid=f.construction_gate(root)
    monkeypatch.setattr(h,'STAGE',root);monkeypatch.setattr(h,'official_process_reader_available',lambda:False)
    calls=[]
    def run(command,**kwargs):
        calls.append(command)
        if fault=='timeout':raise subprocess.TimeoutExpired(command,60)
        code=1 if fault=='returncode' else 0
        value=copy.deepcopy(valid)
        if fault=='wrongjob':value['job_id']='old-job'
        if fault=='wrongnewsource':value['new_QA_source_manifest_sha256']='0'*64
        if fault=='wrongoldbinding':value['historical_binding_sha256']='0'*64
        data=json.dumps(value).encode()
        if fault=='oversize':data=b' '*32769
        if fault=='invalidjson':data=b'not json SECRET'
        return types.SimpleNamespace(returncode=code,stdout=data,stderr=b'SECRET')
    monkeypatch.setattr(h.subprocess,'run',run)
    with pytest.raises((legacy.BridgeError,subprocess.TimeoutExpired)) as error:f.construction_gate(root)
    assert len(calls)==1 and 'SECRET' not in str(error.value)
    popen.assert_not_called()


def test_consumed_construction_stays_retired_and_missing_directory_never_recreated(fixed,monkeypatch):
    root,popen=fixed
    monkeypatch.setattr(f,'CONSTRUCTION_RETIRED',True)
    slot=f.FixedConstructionSlot(root)
    assert slot.status()['remaining_attempts']==0
    with pytest.raises(legacy.BridgeError,match='retired'):slot.start(f.START_BUILD)
    with pytest.raises(worker.RuntimeGateError,match='retired'):worker.run_build(slot.safe(slot.ATTEMPT),stage_root=root)
    assert not slot.safe(slot.ATTEMPT).exists()
    popen.assert_not_called()


def test_historical_complete_status_never_restarts_build_and_QA_once_budget_unchanged(dual):
    root,path,output,popen=dual
    slot=f.FixedConstructionSlot(root)
    result=slot.start(f.START_BUILD)
    assert result['status']=='already_reserved' and not result['new_worker_started'] and result['job']['status']=='completed'
    assert slot.status()['remaining_attempts']==0 and f.POLICY['construction_attempts']==0 and f.POLICY['additional_QA_attempts']==0
    assert all(f.FixedQASlot(name,root).status()['remaining_attempts']==1 for name in ('background','arm'))
    assert all(not (root/'runs/mcp_local_quality/batch_001'/name).exists() for name in ('background','arm'))
    popen.assert_not_called()
