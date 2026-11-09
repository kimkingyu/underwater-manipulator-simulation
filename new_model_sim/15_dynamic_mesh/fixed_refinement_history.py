"""Read-only historical build verifier and fixed host-environment dispatch.

Never imports a solver/Workbench launcher or invokes a worker/start. Old build
sources are checked from actual immutable archives, never current QA source code.
"""
from __future__ import annotations
import importlib.util
import json
from pathlib import Path
import subprocess

from mcp_bridge import STAGE, Stage15Bridge, BridgeError, sha256

OLD_GRANT='frozen/fixed_refinement_execution_authorization_001.json'
OLD_MANIFEST='frozen/fixed_refinement_sources_001.json'
HISTORY='frozen/fixed_refinement_completed_build_binding_001.json'
HISTORY_SHA='ffd2811ae7baa17056cd790ca664a678529de7e9b81e8b5a0ded762da19e0930'
HOST_VERIFY_SECONDS=60
MAX_RESPONSE_BYTES=32768


def official_process_reader_available():
    return importlib.util.find_spec('psutil') is not None


class HistoricalConstructionSlot(Stage15Bridge):
    JOB_ID='fixed-refinement-001-construction'
    ATTEMPT='runs/mcp_fixed_refinement/trial_001/construction'
    OPERATION='fixed_refinement_construction'
    AUTHORIZATION=OLD_GRANT
    SOURCE_MANIFEST=OLD_MANIFEST
    def _sources(self):return self._json(OLD_MANIFEST)['files']
    def authorize(self):
        if sha256(self.safe(HISTORY))!=HISTORY_SHA:raise BridgeError('Actual historical construction binding changed')
        binding=self._json(HISTORY)
        if (binding.get('job_id')!=self.JOB_ID or binding.get('old_authorization_relative')!=OLD_GRANT or
                binding.get('old_source_manifest_relative')!=OLD_MANIFEST or binding.get('old_source_count')!=29 or
                binding.get('additional_construction_attempts')!=0 or binding.get('additional_QA_attempts')!=0 or
                sha256(self.safe(OLD_GRANT))!=binding.get('old_authorization_sha256') or
                sha256(self.safe(OLD_MANIFEST))!=binding.get('old_source_manifest_sha256')):
            raise BridgeError('Actual completed build original grant/29source manifest mismatch')
        grant=self._json(OLD_GRANT)
        if (grant.get('construction_job_id')!=self.JOB_ID or grant.get('construction_attempts')!=1 or
                grant.get('construction_total_seconds')!=7200 or grant.get('QA_attempts_per_domain')!=1 or
                grant.get('source_manifest_relative')!=OLD_MANIFEST or
                grant.get('source_manifest_sha256')!=binding['old_source_manifest_sha256'] or len(self._sources())!=29):
            raise BridgeError('Historical actual fixed trial scope differs')
        paths=historical_paths(self)
        rows=binding.get('files')
        if type(rows) is not list or len(rows)!=len(paths) or {r.get('path') for r in rows}!=set(paths):
            raise BridgeError('Historical actual structured artifact set incomplete')
        for row in rows:
            path=self.safe(row['path'])
            if path.stat().st_size!=row.get('size') or sha256(path)!=row.get('sha256'):
                raise BridgeError('Original completed build artifact changed')
        return grant


def historical_paths(slot):
    from fixed_refinement_bridge import USER_AUTH, MESH_ATTEMPT
    names=['request.json','probe_result.json','launcher.json','worker_claim.json',
           'source_snapshot/archive_manifest.json','source_snapshot/execution_binding.json']
    result=[slot.ATTEMPT+'/'+name for name in names]
    for row in slot._sources():
        path=Path(row['path'])
        try:relative=path.relative_to(slot.root).as_posix()
        except ValueError:
            if path!=slot.root.parent/'04_workbench/run_wb_fullenv.py':raise BridgeError('Unexpected old external build source')
            relative='external/run_wb_fullenv.py'
        result.append(slot.ATTEMPT+'/source_snapshot/'+relative)
    result += [slot.ATTEMPT+'/source_snapshot/execution_binding/'+Path(rel).name for rel in (USER_AUTH,OLD_GRANT,OLD_MANIFEST)]
    result += [MESH_ATTEMPT+'/'+name for name in ('mesh_stage15.wbjn','mesh_plan.json','fixed_refinement_configuration.json',
        'workbench_execution.json','workbench_completion.json','background_meshing_stats.json','arm_meshing_stats.json')]
    return result


def host_read_only_gate(stage_root=STAGE):
    """Exactly one fixed official-Python verifier; no free command/path or retry."""
    from fixed_refinement_bridge import FixedQASlot, USER_AUTH, USER_AUTH_SHA
    slot=FixedQASlot('background',stage_root)
    slot.authorize()  # current corrected QA sources independent of the old build
    if slot.root!=STAGE:raise BridgeError('Read-only host dispatch requires fixed production workspace')
    if sha256(slot.safe(USER_AUTH))!=USER_AUTH_SHA:raise BridgeError('Actual user trial grant changed')
    command=[slot._python(),'-B',str(slot.safe('fixed_refinement_history.py')),'--verify-completed-construction']
    result=subprocess.run(command,stdin=subprocess.DEVNULL,capture_output=True,timeout=HOST_VERIFY_SECONDS,
        cwd=str(slot.root),env=slot._environment(),shell=False)
    if result.returncode!=0 or len(result.stdout)>MAX_RESPONSE_BYTES:
        raise BridgeError('Fixed read-only completed-build verifier blocked; no native launch')
    try:value=json.loads(result.stdout.decode('utf-8-sig'))
    except (UnicodeError,ValueError) as exc:raise BridgeError('Read-only verifier returned invalid bounded JSON') from exc
    if (type(value) is not dict or value.get('status')!='passed' or value.get('job_id')!='fixed-refinement-001-construction' or
            value.get('new_QA_source_manifest_sha256')!=sha256(slot.safe(slot.SOURCE_MANIFEST)) or
            value.get('historical_binding_sha256')!=HISTORY_SHA):
        raise BridgeError('Read-only verifier evidence/source/identity mismatch')
    return value


if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--verify-completed-construction',action='store_true',required=True)
    parser.parse_args()
    if not official_process_reader_available():
        print(json.dumps({'status':'blocked','error_class':'OfficialProcessReaderUnavailable','native_started':False}))
        raise SystemExit(1)
    try:
        from fixed_refinement_bridge import construction_gate
        value=construction_gate()
        print(json.dumps(value,ensure_ascii=False,allow_nan=False))
    except Exception as exc:
        print(json.dumps({'status':'blocked','error_class':type(exc).__name__,'native_started':False}))
        raise SystemExit(1)
