"""Independent one-build/two-QA fixed refinement; no parameterized/sweep inputs."""
from __future__ import annotations
import math
from pathlib import Path
import shutil
import subprocess
import time

from mcp_bridge import STAGE, Stage15Bridge, BridgeError, sha256, write_new
from inspection_batch002_bridge import Batch002Slot
from mesh_mcp_bridge import MeshSlot
from fixed_refinement_config import TRIAL, appended_spheres, BASELINE_MESHES, trial_settings, validate_overlay, validate_native_spheres

USER_AUTH = "frozen/fixed_refinement_trial_authorization_001.json"
USER_AUTH_SHA = "4175f9d7e36d5b3a6afff8da84c4a7131ed28e8204ad1bb79ddc290129fdd2f9"
GRANT = "frozen/fixed_refinement_execution_authorization_001.json"
MANIFEST = "frozen/fixed_refinement_sources_001.json"
BUILD_JOB = "fixed-refinement-001-construction"
BUILD_ATTEMPT = "runs/mcp_fixed_refinement/trial_001/construction"
MESH_ATTEMPT = "runs/mesh/fixed_refinement_trial_001"
START_BUILD = "RUN_STAGE15_FIXED_REFINEMENT_CONSTRUCTION_ONCE"
START_QA = "RUN_STAGE15_FIXED_REFINEMENT_QA_ONCE"
CANCEL_BUILD = "CANCEL_STAGE15_FIXED_REFINEMENT_CONSTRUCTION"
CANCEL_QA = "CANCEL_STAGE15_FIXED_REFINEMENT_QA"
PROVENANCE = "live_fluent_fixed_refinement_trial001"
SOURCES = ("stage15_mcp.py", "mcp_bridge.py", "mesh_mcp_bridge.py", "mesh_construction.py", "mesh_prepare.py",
    "run_mesh_job.py", "runtime.py", "preflight.py", "definition_probe.py", "sizing_mcp_bridge.py",
    "inspection_batch002.py", "inspection_batch002_bridge.py", "inspection_batch002_connection.py",
    "inspection_full_record.py", "inspection_license_review.py", "mesh_inspection_mcp_bridge.py",
    "mesh_inspection_connection.py", "mesh_inspection_evidence.py", "fixed_refinement_config.py",
    "fixed_refinement_bridge.py", "fixed_refinement_construction.py", "local_quality_diagnostic.py", "local_quality_evidence.py",
    "frozen/local_quality_api_basis_001.json", "mesh/meshing_stage15.py.in", "mesh/wb_stage15.wbjn.in",
    "contract.json", "frozen/input_manifest.json")
POLICY = {"schema_version": 1, "trial_id": TRIAL, "construction_job_id": BUILD_JOB,
    "construction_attempt_relative": BUILD_ATTEMPT, "mesh_attempt_relative": MESH_ATTEMPT,
    "construction_attempts": 1, "construction_total_seconds": 7200, "construction_domain_seconds": 3600,
    "QA_domains": ["background", "arm"], "QA_attempts_per_domain": 1, "QA_total_seconds": 300,
    "cleanup_reserve_seconds": 30, "appended_spheres": appended_spheres(), "baseline_meshes": BASELINE_MESHES,
    "baseline_control_changes_allowed": False, "original_CAD_or_mesh_changes_allowed": False,
    "contract_changes_allowed": False, "automatic_retry_allowed": False, "additional_connection_probes": 0,
    "additional_local_diagnostics": 0, "initialization_allowed": False, "iterations_allowed": False,
    "motion_allowed": False, "overset_assembly_allowed": False, "CFD_allowed": False,
    "quality_minimum_required": 0.01, "source_manifest_relative": MANIFEST}


def authorize(slot):
    if sha256(slot.safe(USER_AUTH)) != USER_AUTH_SHA:
        raise BridgeError("Actual fixed trial user authorization changed")
    actual = slot._json(USER_AUTH)
    if (actual.get("trial_id") != TRIAL or actual.get("construction_attempts") != 1 or
            actual.get("QA_attempts_per_domain") != 1 or actual.get("appended_spheres") != appended_spheres() or
            actual.get("construction_total_seconds_including_cleanup") != 7200 or
            actual.get("QA_seconds_including_cleanup") != 300 or actual.get("automatic_retries") != 0):
        raise BridgeError("Actual fixed trial scope invalid")
    grant = slot._json(GRANT)
    metadata = {"authorization_source", "recorded_local", "execution_policy", "source_manifest_sha256"}
    if set(grant) != set(POLICY) | metadata:
        raise BridgeError("Fixed execution authorization fields differ")
    for key, expected in POLICY.items():
        if type(grant.get(key)) is not type(expected) or grant[key] != expected:
            raise BridgeError("Fixed execution policy mismatch: " + key)
    if any(type(grant[k]) is not str or not 1 <= len(grant[k]) <= 4000 for k in metadata - {"source_manifest_sha256"}):
        raise BridgeError("Fixed trial provenance missing")
    if sha256(slot.safe(MANIFEST)) != grant["source_manifest_sha256"] or slot._json(MANIFEST) != {"schema_version": 1, "files": slot._sources()}:
        raise BridgeError("New fixed trial source binding changed")
    for row in BASELINE_MESHES.values():
        path = slot.safe(row["relative"])
        if path.stat().st_size != row["size"] or sha256(path) != row["sha256"]:
            raise BridgeError("Original attempt004 mesh changed")
    from preflight import verify_inputs
    inputs = verify_inputs(slot._json("frozen/input_manifest.json"), workspace=slot.root.parent.parent)
    if inputs.get("status") != "passed" or len(inputs.get("files", [])) != 8:
        raise BridgeError("Eight protected inputs changed")
    return grant


def preserve_binding(slot, request):
    slot.archive_sources(request)
    rows = []
    for relative in (USER_AUTH, GRANT, MANIFEST):
        source = slot.safe(relative)
        target = slot.safe(slot.ATTEMPT + "/source_snapshot/execution_binding/" + Path(relative).name)
        target.parent.mkdir(parents=True, exist_ok=True)
        with source.open("rb") as original, target.open("xb") as saved: shutil.copyfileobj(original, saved)
        if sha256(target) != sha256(source): raise BridgeError("Fixed trial archived authorization changed")
        rows.append({"original": relative, "archive_relative": "execution_binding/" + Path(relative).name, "sha256": sha256(source)})
    write_new(slot.safe(slot.ATTEMPT + "/source_snapshot/execution_binding.json"), {"job_id": slot.JOB_ID, "files": rows})


def validate_source_archive(slot, request):
    manifest = slot._json(slot.ATTEMPT + "/source_snapshot/archive_manifest.json")
    files = manifest.get("files", [])
    if manifest.get("job_id") != slot.JOB_ID or len(files) != len(request["sources"]):
        raise BridgeError("Fixed execution source archive incomplete")
    expected_paths = set()
    for archived, row in zip(files, request["sources"]):
        source = Path(row["path"])
        try: expected = source.relative_to(slot.root).as_posix()
        except ValueError:
            if source != slot.root.parent / "04_workbench/run_wb_fullenv.py": raise BridgeError("Unexpected external source")
            expected = "external/run_wb_fullenv.py"
        if (archived.get("archive_relative") != expected or archived.get("original_path") != row["path"] or
                archived.get("sha256") != row["sha256"] or archived.get("size") != row["size"] or expected in expected_paths):
            raise BridgeError("Fixed source archive row mismatch")
        path = slot.safe(slot.ATTEMPT + "/source_snapshot/" + expected)
        if path.stat().st_size != row["size"] or sha256(path) != row["sha256"]: raise BridgeError("Archived source changed")
        expected_paths.add(expected)
    bindings = [{"original": rel, "archive_relative": "execution_binding/" + Path(rel).name,
                 "sha256": sha256(slot.safe(rel))} for rel in (USER_AUTH, GRANT, MANIFEST)]
    if slot._json(slot.ATTEMPT + "/source_snapshot/execution_binding.json") != {"job_id": slot.JOB_ID, "files": bindings}:
        raise BridgeError("Fixed archived execution bindings differ")
    for row in bindings:
        if sha256(slot.safe(slot.ATTEMPT + "/source_snapshot/" + row["archive_relative"])) != row["sha256"]:
            raise BridgeError("Archived trial binding changed")


class FixedConstructionSlot(Stage15Bridge):
    JOB_ID = BUILD_JOB
    ATTEMPT = BUILD_ATTEMPT
    AUTHORIZATION = GRANT
    SOURCES = SOURCES
    WORKER_SCRIPT = "fixed_refinement_construction.py"
    OPERATION = "fixed_refinement_construction"
    CANCEL_OPERATION = "cancel_fixed_refinement_construction"
    START_CONFIRMATION = START_BUILD
    CANCEL_CONFIRMATION = CANCEL_BUILD
    TOTAL_SECONDS = 7200
    CLEANUP_RESERVE_SECONDS = 30
    LOGS = dict(MeshSlot.LOGS)

    def authorize(self): return authorize(self)
    def preserve_sources(self, request): return preserve_binding(self, request)
    def log_path(self, stream):
        return self.safe((self.ATTEMPT if stream == "worker" else MESH_ATTEMPT) + "/" + self.LOGS[stream])
    def preflight(self):
        self.authorize()
        output = subprocess.run([self._python(), "-B", str(self.safe(self.WORKER_SCRIPT)), "--preflight"],
            stdin=subprocess.DEVNULL, capture_output=True, timeout=45, cwd=str(self.root), env=self._environment(), shell=False)
        if output.returncode != 0 or len(output.stdout) > 65536: raise BridgeError("Fixed construction preflight failed")
        result = __import__("json").loads(output.stdout.decode("utf-8-sig"))
        if result.get("status") != "passed": raise BridgeError("Fixed construction blocked")
        return result
    def start(self, confirmation):
        if confirmation != START_BUILD: raise BridgeError("Explicit fixed construction confirmation required")
        if not self.safe(self.ATTEMPT).exists():
            self.authorize()
            if self.safe(MESH_ATTEMPT).exists(): raise BridgeError("Fixed trial output already exists; never adopt or overwrite")
            for lock in (".ansys-task.lock", ".sizing-dispatch.lock", ".mesh-inspection-dispatch.lock"):
                if self.safe(lock).exists(): raise BridgeError("Existing task/dispatch lock blocks fixed trial")
            self.preflight()
        if self.safe(self.ATTEMPT).exists():
            return {'status':'already_reserved','new_worker_started':False,'job':self.job_status(self.JOB_ID),'trial_id':TRIAL}
        import uuid
        token=uuid.uuid4().hex
        dispatch=self.safe('.mesh-inspection-dispatch.lock')
        try:write_new(dispatch,{'job_id':self.JOB_ID,'token':token,'created_unix':time.time()})
        except FileExistsError:raise BridgeError('Existing serial dispatch; fixed build slot not consumed')
        result=None
        try:
            result = Stage15Bridge.start(self, confirmation)
            result.pop("remaining_definition_probes", None)
            result.update(trial_id=TRIAL, CFD_authorized=False, mesh_attempt_relative=MESH_ATTEMPT)
            return result
        finally:
            release=(result is not None and result.get('owned_cleanup_verified') is True) or not self.safe(self.ATTEMPT).exists()
            if release and dispatch.exists() and self._json(dispatch.name).get('token')==token:dispatch.unlink()
    def status(self):
        used = self.safe(self.ATTEMPT).exists()
        result = {"trial_id": TRIAL, "job_id": self.JOB_ID, "slot_reserved": used, "remaining_attempts": 0 if used else 1,
            "maximum_seconds_including_cleanup":7200, "domain_seconds":3600, "mesh_attempt_relative":MESH_ATTEMPT,
            "start_confirmation":START_BUILD,"cancel_confirmation":CANCEL_BUILD,"CFD_authorized":False}
        if used: result["job"] = self.job_status(self.JOB_ID)
        return result
    def job_status(self, job_id):
        result = Stage15Bridge.job_status(self, job_id)
        result.pop("remaining_definition_probes", None)
        result.update(trial_id=TRIAL, CFD_authorized=False, mesh_motion_verified=False)
        final = result.get("result")
        if isinstance(final, dict) and final.get("status") == "completed":
            try:
                result["construction_binding"] = construction_gate(self.root)
            except Exception:
                result.update(status="failed", construction_binding={"status":"unverified"})
        return result


def validate_native_controls(config, stats):
    """Actual base and sphere settings must agree; configuration text alone is insufficient."""
    from fixed_refinement_config import quantity_metres
    validate_native_spheres(config, stats)
    ids=stats.get('fluid_body_geometry_ids')
    if type(ids) is not list or len(ids)!=1 or type(ids[0]) is not int or ids[0]<=0 or any(row['geometry_ids']!=ids for row in stats['sphere_readback']):
        raise BridgeError('Actual sphere scoping must equal the sole fluid body geometry ID')
    rb=stats.get('readback',{})
    for native,field in (('ElementSize','global_size_m'),('MaximumSize','maximum_size_m'),('MinimumSize','minimum_size_m')):
        if not math.isclose(quantity_metres(rb.get(native)),config[field],rel_tol=1e-8,abs_tol=1e-12):
            raise BridgeError('Baseline native mesh sizing changed: '+native)
    try:growth=float(rb.get('GrowthRate'))
    except (ValueError,TypeError):raise BridgeError('Native global growth unavailable')
    if (not math.isfinite(growth) or growth!=1.3 or rb.get('CaptureCurvature')!='False' or rb.get('CaptureProximity')!='False' or
            stats.get('tet_method_readback')!='AllTriAllTet'):
        raise BridgeError('Baseline native growth/capture/tet method differs')
    local=stats.get('local_readback')
    if type(local) is not list or len(local)!=len(config['face_controls']):raise BridgeError('Actual face sizing records incomplete')
    for row,expected in zip(local,config['face_controls']):
        if (row.get('name')!='stage15_'+expected['name'] or row.get('source_named_selection')!=expected['name'] or
                row.get('local_growth_override_applied') is not False or row.get('global_growth_rate_requested')!=1.3 or
                not math.isclose(quantity_metres(row.get('size')),expected['size_m'],rel_tol=1e-8,abs_tol=1e-12) or
                not row.get('expected_geometry_ids') or row.get('actual_geometry_ids')!=row.get('expected_geometry_ids') or
                row.get('definition',{}).get('properties',{}).get('ObjectState')!='FullyDefined'):
            raise BridgeError('Actual baseline local face sizing/scope/state changed')
    return True


def construction_gate(stage_root=STAGE):
    """No native launch. Actual sphere settings, source archives and new mesh SHA required."""
    slot = FixedConstructionSlot(stage_root)
    slot.authorize()
    request = slot._json(slot.ATTEMPT + "/request.json")
    validate_build_request(slot, request)
    final = slot._json(slot.ATTEMPT + "/probe_result.json")
    created = request["created_unix"]
    if (final.get("job_id") != BUILD_JOB or final.get("trial_id") != TRIAL or final.get("status") != "completed" or
            final.get("operation") != slot.OPERATION or any(final.get(k) is not True for k in
            ("owned_cleanup_verified", "inputs_reverified_after_cleanup", "sources_reverified_after_cleanup", "serial_dispatch_released", "native_spheres_verified",
             "mesh_exports_written", "mesh_generated")) or final.get("solver_launched") is not False or
            type(final.get("flow_iterations")) is not int or final["flow_iterations"] != 0 or
            final.get("mesh_motion_verified") is not False or type(final.get("workbench_exit_code")) is not int or final.get("workbench_exit_code") != 0 or
            any(final.get(k) != request[k] for k in ("created_unix", "deadline_unix", "work_deadline_unix")) or
            type(final.get("elapsed_seconds")) not in (int,float) or not math.isfinite(final["elapsed_seconds"]) or
            not 0 < final["elapsed_seconds"] <= 7200 or created + final["elapsed_seconds"] > time.time()+1 or
            type(final.get('finished_unix')) not in (int,float) or not math.isfinite(final['finished_unix']) or
            not created<=final['finished_unix']<=request['deadline_unix'] or abs(final['finished_unix']-created-final['elapsed_seconds'])>2):
        raise BridgeError("Fixed construction execution/input/cleanup/deadline evidence incomplete")
    from definition_probe import validate_frozen
    inputs = validate_frozen(stage_root)["protected_inputs"]
    if final.get("protected_inputs_before") != inputs or final.get("protected_inputs_after") != inputs:
        raise BridgeError("Fixed construction original eight-input binding changed")
    validate_source_archive(slot, request)
    plan = slot._json(MESH_ATTEMPT + "/mesh_plan.json")
    configs = trial_settings(slot._json("contract.json"), slot.safe(MESH_ATTEMPT))
    # mesh_prepare adds exact source CAD path into its plan, nothing else may differ.
    from definition_probe import validate_frozen as frozen_inputs
    cad = frozen_inputs(stage_root)["cad"]
    for config in configs: config["cad_path"] = cad[config["domain"]]["path"]
    if plan.get("domains") != configs:
        raise BridgeError("Fixed trial actual plan differs from baseline plus one sphere")
    journal = slot.safe(MESH_ATTEMPT + "/mesh_stage15.wbjn")
    if plan.get("journal",{}).get("sha256") != sha256(journal): raise BridgeError("Fixed generated journal changed")
    execution = slot._json(MESH_ATTEMPT + "/workbench_execution.json")
    if (execution.get("owned_cleanup_verified") is not True or type(execution.get("exit_code")) is not int or execution["exit_code"]!=0 or
            execution.get('solver_launched') is not False or execution.get('automatic_retries')!=0 or
            execution.get('total_timeout_seconds')!=7200 or execution.get('system_timeout_seconds')!=3600 or
            execution.get('total_deadline_unix')!=request['deadline_unix'] or execution.get('work_deadline_unix')!=request['work_deadline_unix'] or
            execution.get('journal_sha256')!=sha256(journal) or execution.get('attempt')!=str(slot.safe(MESH_ATTEMPT))):
        raise BridgeError("Fixed Workbench cleanup/exit/original budget record incomplete")
    completion=slot._json(MESH_ATTEMPT+'/workbench_completion.json')
    from mesh_construction import validate_exports
    exports=validate_exports(slot.safe(MESH_ATTEMPT),completion)
    for relative,key in ((MESH_ATTEMPT+'/mesh_stage15.wbjn','journal_sha256'),(MESH_ATTEMPT+'/workbench_completion.json','completion_sha256'),
        (MESH_ATTEMPT+'/workbench_execution.json','execution_sha256'),(MESH_ATTEMPT+'/mesh_plan.json','plan_sha256'),
        (MESH_ATTEMPT+'/fixed_refinement_configuration.json','comparison_sha256')):
        if final.get(key)!=sha256(slot.safe(relative)):raise BridgeError('Fixed build artifact hash changed: '+key)
    from fixed_refinement_config import configuration_comparison
    if slot._json(MESH_ATTEMPT+'/fixed_refinement_configuration.json')!=configuration_comparison(slot._json('contract.json'),slot.safe(MESH_ATTEMPT)):
        raise BridgeError('Fixed baseline overlay audit changed')
    rows = final.get("meshes")
    if type(rows) is not list or [x.get("domain") for x in rows] != ["background","arm"]:
        raise BridgeError("Exactly two new trial mesh records required")
    records = []
    for config, row in zip(configs, rows):
        domain = config["domain"]
        stats = slot._json(MESH_ATTEMPT + "/" + domain + "_meshing_stats.json")
        validate_native_controls(config, stats)
        if stats.get("settings") != config or stats.get("mesh_generated") is not True or any(type(stats.get(k)) is not int or stats[k]<=0 for k in ("nodes","elements")):
            raise BridgeError("Fixed native settings/counts binding missing")
        path = slot.safe(row.get("relative", "invalid"))
        if path != exports[len(records)]:raise BridgeError('Fixed output not bound to its actual Workbench export')
        from definition_probe import checked_path
        source=checked_path(completion['systems'][len(records)]['mesh_export']['source_path'],slot.safe(MESH_ATTEMPT))
        if sha256(source)!=row.get('sha256'):raise BridgeError('Native Workbench export and independent trial copy differ')
        if path.parent != slot.safe(MESH_ATTEMPT) or path.name not in (domain+".msh",domain+".msh.h5"):
            raise BridgeError("Fixed mesh must be the new independent output only")
        if type(row.get("size")) is not int or row["size"]<=0 or path.stat().st_size!=row["size"] or sha256(path)!=row.get("sha256"):
            raise BridgeError("New fixed mesh changed")
        if row.get("stats_sha256") != sha256(slot.safe(MESH_ATTEMPT + "/" + domain + "_meshing_stats.json")):
            raise BridgeError("Actual sphere readback statistics changed")
        records.append(row)
    return {"status":"passed", "trial_id":TRIAL, "job_id":BUILD_JOB, "construction_result_sha256":sha256(slot.safe(slot.ATTEMPT+"/probe_result.json")),
        "construction_request_sha256":sha256(slot.safe(slot.ATTEMPT+"/request.json")), "source_manifest_sha256":sha256(slot.safe(MANIFEST)),
        "grant_sha256":sha256(slot.safe(GRANT)), "journal_sha256":sha256(journal), "meshes":records}


def validate_build_request(slot, request):
    if (type(request) is not dict or set(request) != {"schema_version","operation","job_id","created_unix","deadline_unix","work_deadline_unix","authorization","sources"} or
            type(request.get("schema_version")) is not int or request["schema_version"]!=1 or request.get("job_id")!=BUILD_JOB or request.get("operation")!=slot.OPERATION or
            request.get("sources")!=slot._sources() or request.get("authorization")!={"path":str(slot.safe(GRANT)),"sha256":sha256(slot.safe(GRANT))}):
        raise BridgeError("Fixed construction request identity/source mismatch")
    created=request.get("created_unix")
    if (any(type(request.get(k)) not in (int,float) or not math.isfinite(request[k]) for k in ("created_unix","deadline_unix","work_deadline_unix")) or
            created>time.time()+1 or request["deadline_unix"]!=created+7200 or request["work_deadline_unix"]!=created+7170):
        raise BridgeError("Fixed construction original deadline invalid")
    return request


class FixedQASlot(Batch002Slot):
    AUTHORIZATION = GRANT
    SOURCE_MANIFEST = MANIFEST
    SOURCES = SOURCES
    WORKER_SCRIPT = "fixed_refinement_construction.py"
    START_CONFIRMATION = START_QA
    CANCEL_CONFIRMATION = CANCEL_QA
    CANCEL_OPERATION = "cancel_fixed_refinement_QA"
    OPERATION = "mesh_inspection"
    def __init__(self, domain, stage_root=STAGE):
        if domain not in ("background","arm"): raise BridgeError("Only fixed trial background/arm QA accepted")
        Stage15Bridge.__init__(self,stage_root)
        self.slot=self.domain=domain
        self.is_connection=False
        self.JOB_ID="fixed-refinement-001-QA-"+domain
        self.ATTEMPT="runs/mcp_fixed_refinement/trial_001/QA_"+domain
    def authorize(self): return authorize(self)
    def preserve_sources(self, request): return preserve_binding(self,request)
    def connection_gate(self): return construction_gate(self.root)
    def mesh_input(self):
        from inspection_batch002_bridge import MESHES
        row=next(x for x in self.connection_gate()["meshes"] if x["domain"]==self.domain)
        return dict(MESHES[self.domain],path=row["relative"],size=row["size"],sha256=row["sha256"])
    def make_request(self,created,sources):
        value=Stage15Bridge.make_request(self,created,sources)
        value.update(batch_id=TRIAL,slot=self.slot,mesh=self.mesh_input(),construction=self.connection_gate())
        return value
    def preflight(self):
        self.authorize();self.connection_gate()
        result=subprocess.run([self._python(),"-B",str(self.safe(self.WORKER_SCRIPT)),"--QA-preflight","--domain",self.domain],
            stdin=subprocess.DEVNULL,capture_output=True,timeout=60,cwd=str(self.root),env=self._environment(),shell=False)
        if result.returncode!=0 or len(result.stdout)>65536: raise BridgeError("Fixed QA read-only preflight blocked")
        value=__import__('json').loads(result.stdout.decode('utf-8-sig'))
        if value.get('status')!='passed': raise BridgeError('Fixed QA preflight not passed')
        return value
    def start(self,confirmation):
        value=Batch002Slot.start(self,confirmation);value.update(trial_id=TRIAL,batch_id=TRIAL,CFD_authorized=False);return value
    def status(self):
        used=self.safe(self.ATTEMPT).exists()
        result={"trial_id":TRIAL,"job_id":self.JOB_ID,"domain":self.domain,"slot_reserved":used,"remaining_attempts":0 if used else 1,
            "maximum_seconds_including_cleanup":300,"start_confirmation":START_QA,"cancel_confirmation":CANCEL_QA,"CFD_authorized":False}
        if used:result['job']=self.job_status(self.JOB_ID)
        return result
    def job_status(self,job_id):
        value=Stage15Bridge.job_status(self,job_id);value.pop('remaining_definition_probes',None);value.update(trial_id=TRIAL,CFD_authorized=False)
        final=value.get('result')
        if isinstance(final,dict):
            from mesh_inspection_evidence import validate_native_evidence,parse_native_output
            from inspection_full_record import verify_full_record_artifacts
            from inspection_batch002_connection import readiness_passed
            bound=False
            try:
                native=self._json(self.ATTEMPT+'/native_result.json');request=self._json(self.ATTEMPT+'/request.json');validate_QA_request(self,request)
                transcript=self.safe(self.ATTEMPT+'/native.transcript.txt');ready=self._json(self.ATTEMPT+'/readiness.json')
                verify_full_record_artifacts(self,native,final,classified=True)
                bound=(final.get('native')==native and native.get('provenance')==PROVENANCE and native.get('job_id')==self.JOB_ID and
                    native.get('batch_id')==TRIAL and native.get('slot')==self.slot and readiness_passed(ready) and native.get('readiness')==ready and
                    native.get('readiness_sha256')==sha256(self.safe(self.ATTEMPT+'/readiness.json')) and 0<transcript.stat().st_size<=2097152 and
                    native.get('transcript_sha256')==sha256(transcript) and native.get('mesh_evidence',{}).get('metrics')==parse_native_output(transcript.read_text(encoding='utf-8',errors='replace')) and
                    final.get('native_adapter_exit_code')==0 and all(final.get(k) is True for k in ('owned_cleanup_verified','inputs_reverified_after_cleanup','sources_reverified_after_cleanup','serial_dispatch_released')))
                validate_source_archive(self,request)
            except Exception: pass
            value['inspection']=validate_native_evidence(final.get('native',{}).get('mesh_evidence',{}) if bound else {},self.domain)
            value['inspection']['native_artifacts_bound']=bound
            value['mesh_motion_verified']=False
        return value


def validate_QA_request(slot,value):
    keys={"schema_version","operation","job_id","created_unix","deadline_unix","work_deadline_unix","authorization","sources","batch_id","slot","mesh","construction"}
    if (type(value) is not dict or set(value)!=keys or type(value.get('schema_version')) is not int or value['schema_version']!=1 or
            value.get('job_id')!=slot.JOB_ID or value.get('operation')!=slot.OPERATION or value.get('batch_id')!=TRIAL or value.get('slot')!=slot.slot or
            value.get('mesh')!=slot.mesh_input() or value.get('construction')!=slot.connection_gate() or value.get('sources')!=slot._sources() or
            value.get('authorization')!={"path":str(slot.safe(GRANT)),"sha256":sha256(slot.safe(GRANT))}):
        raise BridgeError('Fixed QA request not bound to this new construction/source/mesh')
    c=value.get('created_unix')
    if (any(type(value.get(k)) not in (int,float) or not math.isfinite(value[k]) for k in ('created_unix','deadline_unix','work_deadline_unix')) or
            c>time.time()+1 or value['deadline_unix']!=c+300 or value['work_deadline_unix']!=c+270): raise BridgeError('Fixed QA original deadline invalid')
    return value


def validate_QA_attempt(slot,attempt):
    value=slot._json(slot.ATTEMPT+'/request.json')
    if Path(attempt)!=slot.safe(slot.ATTEMPT):raise BridgeError('Fixed QA attempt path mismatch')
    return validate_QA_request(slot,value)


def QA_slot_for_attempt(attempt,stage_root=STAGE):
    from definition_probe import checked_path
    path=checked_path(attempt,stage_root)
    for name in ('background','arm'):
        item=FixedQASlot(name,stage_root)
        if path==item.safe(item.ATTEMPT):return item,path
    raise BridgeError('Only the fixed new trial QA paths accepted')


def fixed_job(job_id,stage_root=STAGE):
    for item in (FixedConstructionSlot(stage_root),FixedQASlot('background',stage_root),FixedQASlot('arm',stage_root)):
        if item.JOB_ID==job_id:return item
    raise BridgeError('Unknown fixed trial job ID')
