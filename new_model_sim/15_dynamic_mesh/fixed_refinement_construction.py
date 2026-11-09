"""Fixed-refinement build/QA workers. No execution on import or preflight.

The native build is delegated only after an explicit reserved new slot. Existing
Workbench runner owns every child, deadlines and cleanup; no new launch path.
"""
from __future__ import annotations
import argparse
import json
import os
from pathlib import Path
import time
import sys
sys.dont_write_bytecode=True
import psutil
import definition_probe as probe
import mesh_prepare
import run_mesh_job
import mesh_construction as original
from runtime import RuntimeGateError
from mcp_bridge import STAGE, sha256, read_json, write_new
from fixed_refinement_config import TRIAL, trial_settings, validate_native_spheres, validate_overlay
from fixed_refinement_bridge import (FixedConstructionSlot,FixedQASlot,BUILD_JOB,BUILD_ATTEMPT,MESH_ATTEMPT,
    GRANT,MANIFEST,validate_build_request,validate_QA_request,construction_gate,validate_native_controls)


def environment_clear():
    """Read only process inspection; no lmstat/network/license checkout."""
    from inspection_license_review import NATIVE_NAME
    for process in psutil.process_iter(['pid','name']):
        name=process.info.get('name')
        if not isinstance(name,str):raise RuntimeGateError('Process name unavailable')
        if NATIVE_NAME.fullmatch(name):raise RuntimeGateError('Existing Native client requires serialization')
    return True


def template_preflight(root,frozen):
    # Existing AST audit checks a single GenerateMesh/no local GrowthRate setter.
    result=original.audit_templates(root,frozen)
    configs=trial_settings(frozen['contract'],Path(root)/MESH_ATTEMPT)
    validate_overlay(frozen['contract'],Path(root)/MESH_ATTEMPT,configs)
    template=(Path(root)/'mesh/meshing_stage15.py.in').read_text(encoding='utf-8')
    for config in configs:compile(template.replace('__CONFIG_JSON__',repr(json.dumps(config,ensure_ascii=True))),'<fixed-sphere-in-memory>','exec')
    return dict(result,fixed_refinement_overlay_verified=True,native_readback_verified=False)


def preflight(stage_root=STAGE,domain=None):
    slot=FixedConstructionSlot(stage_root) if domain is None else FixedQASlot(domain,stage_root)
    slot.authorize()
    frozen=probe.bounded_call(lambda:probe.validate_frozen(stage_root),15)
    for lock in ('.ansys-task.lock','.sizing-dispatch.lock','.mesh-inspection-dispatch.lock'):
        if slot.safe(lock).exists():raise RuntimeGateError('Existing ownership or dispatch lock blocks trial')
    environment_clear()
    if domain is None:
        if slot.safe(MESH_ATTEMPT).exists() and not slot.safe(slot.ATTEMPT).exists():
            raise RuntimeGateError('Preexisting fixed trial outputs cannot be adopted')
        templates=template_preflight(stage_root,frozen)
    else:
        slot.connection_gate();templates=None
    software=frozen['contract']['software']
    for key in ('python','workbench','fluent'):
        if key in software and not Path(software[key]).is_file():raise RuntimeGateError('Required fixed executable missing: '+key)
    return {'status':'passed','trial_id':TRIAL,'job_id':slot.JOB_ID,'domain':domain,
        'native_started':False,'slot_reserved':slot.safe(slot.ATTEMPT).exists(),'template_preflight':templates,
        'quality_pass_required_for_QA':False,'CFD_authorized':False}


def cancelled(slot):
    path=slot.safe(slot.ATTEMPT+'/cancel_request.json')
    if not path.exists():return False
    row=read_json(path,4096)
    if row.get('job_id')!=slot.JOB_ID or row.get('operation')!=slot.CANCEL_OPERATION:
        raise RuntimeGateError('Trial cancel request identity mismatch')
    return True


def validate_launcher(slot,request,remaining):
    end=time.monotonic()+min(20,remaining())
    while time.monotonic()<end:
        path=slot.safe(slot.ATTEMPT+'/launcher.json')
        if path.exists():
            ledger=read_json(path,8192)
            if (ledger.get('job_id')!=slot.JOB_ID or ledger.get('pid')!=os.getpid() or
                    ledger.get('creation_time')!=psutil.Process(os.getpid()).create_time()):
                raise RuntimeGateError('Fixed construction worker PID/create-time mismatch')
            return ledger
        time.sleep(min(.05,max(0,end-time.monotonic())))
    raise RuntimeGateError('Fixed construction launcher ledger missing')


def verify_plan(slot,frozen):
    attempt=slot.safe(MESH_ATTEMPT)
    plan=read_json(attempt/'mesh_plan.json')
    expected=trial_settings(frozen['contract'],attempt)
    for config in expected:config['cad_path']=frozen['cad'][config['domain']]['path']
    if plan.get('domains')!=expected:raise RuntimeGateError('Actual generated trial config differs')
    from fixed_refinement_config import configuration_comparison
    if read_json(attempt/'fixed_refinement_configuration.json')!=configuration_comparison(frozen['contract'],attempt):
        raise RuntimeGateError('Offline trial configuration audit differs')
    if plan.get('journal',{}).get('sha256')!=sha256(attempt/'mesh_stage15.wbjn'):
        raise RuntimeGateError('Fixed trial generated journal changed')
    return expected


def run_build(attempt,*,stage_root=STAGE):
    slot=FixedConstructionSlot(stage_root)
    attempt=probe.checked_path(attempt,stage_root)
    if attempt!=slot.safe(slot.ATTEMPT) or (attempt/'probe_result.json').exists():
        raise RuntimeGateError('Only the reserved new fixed build, never a replay')
    request=validate_build_request(slot,read_json(attempt/'request.json'))
    if request['deadline_unix']<=time.time():raise RuntimeGateError('Fixed build deadline expired')
    end=time.monotonic()+max(0,request['deadline_unix']-time.time())
    def remaining(cleanup=False):return max(0,min(end-time.monotonic(),request['deadline_unix']-time.time())-(0 if cleanup else 30))
    def call(function,seconds=15,cleanup=False):return probe.bounded_call(function,min(seconds,remaining(cleanup)))
    ledger=validate_launcher(slot,request,remaining)
    dispatch=slot.safe('.mesh-inspection-dispatch.lock')
    dispatch_row=read_json(dispatch,8192)
    if dispatch_row.get('job_id')!=slot.JOB_ID or type(dispatch_row.get('token')) is not str or len(dispatch_row['token'])!=32:
        raise RuntimeGateError('Fixed construction serial dispatch ownership missing')
    dispatch_sha=sha256(dispatch)
    write_new(attempt/'worker_claim.json',{'job_id':BUILD_JOB,'pid':ledger['pid'],'creation_time':ledger['creation_time']})
    report={'job_id':BUILD_JOB,'trial_id':TRIAL,'operation':slot.OPERATION,'status':'failed','mesh_generated':False,
        'mesh_exports_written':False,'native_spheres_verified':False,'mesh_quality_verified':False,'mesh_motion_verified':False,
        'solver_launched':False,'flow_iterations':0,'static_overset':'not_run','motion':'not_run','flow':'not_run',
        'owned_cleanup_verified':False,'inputs_reverified_after_cleanup':False,'sources_reverified_after_cleanup':False,
        'mesh_attempt_relative':MESH_ATTEMPT,'meshes':[],**{k:request[k] for k in ('created_unix','deadline_unix','work_deadline_unix')}}
    def state(**fields):report.update(fields);probe.write_state(attempt,report)
    def check():
        if cancelled(slot):raise probe.ProbeCancelled('Fixed trial cancellation requested')
        if remaining()<=0:raise TimeoutError('Fixed trial 7170-second work deadline reached')
    def recheck():
        slot.authorize();validate_build_request(slot,read_json(attempt/'request.json'))
        return probe.validate_frozen(stage_root)
    output=slot.safe(MESH_ATTEMPT)
    try:
        check();frozen=call(recheck);report['protected_inputs_before']=frozen['protected_inputs']
        call(lambda:template_preflight(stage_root,frozen));environment_clear()
        for name in ('.ansys-task.lock','.sizing-dispatch.lock'):
            if slot.safe(name).exists():raise RuntimeGateError('Preexisting trial ownership/dispatch lock')
        if sha256(dispatch)!=dispatch_sha:raise RuntimeGateError('Fixed construction dispatch changed before launch')
        if output.exists():raise RuntimeGateError('Fixed output already exists; never overwrite/adopt')
        state(status='running',phase='generate_fixed_journal')
        call(lambda:mesh_prepare.generate(output,frozen['cad']['background']['path'],frozen['cad']['arm']['path'],
            cad_verification=frozen['cad']['cad_verification']['path'],contract_path=slot.safe('contract.json'),stage_root=stage_root,fixed_refinement=True))
        configs=call(lambda:verify_plan(slot,frozen))
        check();call(recheck)
        code=run_mesh_job.run(output,total_deadline_unix=request['deadline_unix'],cancel_check=lambda:cancelled(slot),on_progress=lambda value:state(**value))
        report['workbench_exit_code']=code
        execution=call(lambda:read_json(output/'workbench_execution.json'),10,True)
        report['owned_cleanup_verified']=execution.get('owned_cleanup_verified') is True
        if not report['owned_cleanup_verified']:
            report['status']='cleanup_unverified';raise RuntimeGateError('Fixed Workbench cleanup unverified')
        if code!=0 or execution.get('exit_code')!=0:raise RuntimeGateError('Fixed Workbench did not exit0')
        completion=call(lambda:read_json(output/'workbench_completion.json'),10,True)
        paths=call(lambda:original.validate_exports(output,completion),10,True)
        records=[]
        for config,path in zip(configs,paths):
            check();stats_path=output/(config['domain']+'_meshing_stats.json')
            stats=call(lambda p=stats_path:read_json(p),10)
            validate_native_controls(config,stats)
            if stats.get('settings')!=config or stats.get('mesh_generated') is not True or any(type(stats.get(k)) is not int or stats[k]<=0 for k in ('nodes','elements')):
                raise RuntimeGateError('Actual sphere/native settings and counts missing')
            # Bound hashing of output files; no unbounded mesh parse or cell arrays.
            digest=call(lambda p=path:sha256(p),max(0,remaining()))
            source=probe.checked_path(completion['systems'][len(records)]['mesh_export']['source_path'],output)
            if call(lambda p=source:sha256(p),max(0,remaining()))!=digest:
                raise RuntimeGateError('Actual native export differs from independent trial copy')
            records.append({'domain':config['domain'],'relative':path.relative_to(slot.root).as_posix(),'size':path.stat().st_size,'sha256':digest,
                'nodes':stats['nodes'],'elements':stats['elements'],'stats_sha256':sha256(stats_path),'native_spheres_verified':True})
        check();call(recheck)
        report.update(meshes=records,mesh_generated=True,mesh_exports_written=True,native_spheres_verified=True,
            journal_sha256=sha256(output/'mesh_stage15.wbjn'),completion_sha256=sha256(output/'workbench_completion.json'),
            execution_sha256=sha256(output/'workbench_execution.json'),plan_sha256=sha256(output/'mesh_plan.json'),
            comparison_sha256=sha256(output/'fixed_refinement_configuration.json'),status='completed',phase='construction_exported_pending_QA')
    except Exception as exc:
        if isinstance(exc,probe.ProbeCancelled):report['status']='cancelled'
        elif report.get('status')!='cleanup_unverified':report['status']='failed'
        report['error_class']=type(exc).__name__
        execution=output/'workbench_execution.json'
        if execution.exists():
            try:report['owned_cleanup_verified']=read_json(execution).get('owned_cleanup_verified') is True
            except Exception:report['owned_cleanup_verified']=False
        else:
            launched=any((output/name).exists() for name in ('workbench.stdout.log','workbench.stderr.log'))
            report['owned_cleanup_verified']=not launched
        if not report['owned_cleanup_verified']:report['status']='cleanup_unverified'
    finally:
        try:
            after=call(recheck,10,True);report['protected_inputs_after']=after['protected_inputs'];report['inputs_reverified_after_cleanup']=True
            call(lambda:slot.authorize(),5,True);report['sources_reverified_after_cleanup']=True
        except Exception as exc:
            report['post_cleanup_recheck_error_class']=type(exc).__name__
            if report['status']!='cleanup_unverified':report['status']='failed'
        if report['owned_cleanup_verified']:
            try:
                if sha256(dispatch)!=dispatch_sha or read_json(dispatch).get('job_id')!=slot.JOB_ID:
                    raise RuntimeGateError('Fixed build dispatch changed; retain unknown lock')
                dispatch.unlink();report['serial_dispatch_released']=True
            except Exception as exc:
                report.update(status='cleanup_unverified',serial_dispatch_released=False,dispatch_error_class=type(exc).__name__)
        report.update(finished_unix=time.time(),elapsed_seconds=max(0,7200-remaining(True)),phase='finished')
        if remaining(True)<=0:report['status']='failed' if report['owned_cleanup_verified'] else 'cleanup_unverified'
        write_new(attempt/'probe_result.json',report);probe.write_state(attempt,report)
    return report


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    modes=parser.add_mutually_exclusive_group(required=True)
    modes.add_argument('--preflight',action='store_true');modes.add_argument('--QA-preflight',action='store_true')
    modes.add_argument('--run',action='store_true');modes.add_argument('--native',action='store_true')
    parser.add_argument('--domain',choices=('background','arm'));parser.add_argument('--attempt',type=Path)
    args=parser.parse_args(argv)
    if args.preflight or args.QA_preflight:
        if args.attempt is not None or args.preflight and args.domain is not None or args.QA_preflight and args.domain is None:
            parser.error('Fixed read-only preflight scope mismatch')
        result=preflight(domain=args.domain)
    else:
        if args.attempt is None or args.domain is not None:parser.error('Execution accepts only reserved fixed path')
        if args.attempt.absolute()==(STAGE/BUILD_ATTEMPT).absolute():
            if args.native:parser.error('Construction does not accept Fluent native mode')
            result=run_build(args.attempt)
        else:
            from inspection_batch002 import run,native_collect
            result=native_collect(args.attempt,fixed_refinement=True) if args.native else run(args.attempt,fixed_refinement=True)
    print(json.dumps(result,ensure_ascii=False,allow_nan=False))
    return 0 if args.native or result.get('status') in ('completed','passed') else 1


if __name__=='__main__':raise SystemExit(main())
