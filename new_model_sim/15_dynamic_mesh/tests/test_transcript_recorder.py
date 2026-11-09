"""Offline fixed-path TUI recorder tests; no real API session/stream/process/socket."""
import ast
from collections import defaultdict
import hashlib
import json
import os
from pathlib import Path
import stat
import sys
import types
from unittest import mock

import pytest

STAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(STAGE))
import inspection_batch002 as w
import inspection_batch002_bridge as b
import mcp_bridge as legacy
from test_definition_probe import sandbox, dump
from test_inspection_batch002 import batch, reserve, completed_connection
from test_native_stdin_lifetime import native_input


class SafeFixtureRpcError(RuntimeError):
    def code(self): return types.SimpleNamespace(name="UNKNOWN")


class Repeated(list):
    def add(self):
        value=Variant(); self.append(value); return value
class Variant:
    def __init__(self):
        self.list_value=types.SimpleNamespace(values=Repeated())
        self.struct_value=types.SimpleNamespace(fields=defaultdict(Variant))


SERIALIZER_FALLBACK='''def _convert_value_to_gvalue(val, gval):
    if isinstance(val, bool): gval.bool_value=val
    elif isinstance(val,int) or isinstance(val,float): gval.number_value=val
    elif isinstance(val,str): gval.string_value=val
    elif isinstance(val,list) or isinstance(val,tuple):
        gval.list_value.values.add()
        gval.list_value.values.pop()
        for item in val:
            item_gval=gval.list_value.values.add()
            _convert_value_to_gvalue(item,item_gval)
    elif isinstance(val,dict):
        for k,v in val.items(): _convert_value_to_gvalue(v,gval.struct_value.fields[k])
'''
COMMAND_FALLBACK='''class DatamodelService:
    def execute_command(self,path,*args,**kwargs):
        request=DataModelProtoModule.ExecuteCommandRequest()
        request.path=path
        if kwargs:
            for k,v in kwargs.items(): _convert_value_to_gvalue(v,request.args.fields[k])
        else: _convert_value_to_gvalue(args,request.args.fields["tui_args"])
        return self._impl.execute_command(request)
'''


def official_fake_service():
    """Compile only installed official serializer/method bodies, no package import."""
    path=Path(r"D:/Program Files/ANSYS Inc/v242/commonfiles/CPython/3_10/winx64/Release/python/lib/site-packages/ansys/fluent/core/services/datamodel_tui.py")
    source=path.read_text(encoding="utf-8") if path.is_file() else SERIALIZER_FALLBACK+COMMAND_FALLBACK
    tree=ast.parse(source)
    convert=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=="_convert_value_to_gvalue")
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=="DatamodelService")
    command=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=="execute_command")
    requests=[]
    def request(): return types.SimpleNamespace(path="",args=types.SimpleNamespace(fields=defaultdict(Variant)))
    scope={"DataModelProtoModule":types.SimpleNamespace(ExecuteCommandRequest=request)}
    module=ast.Module(body=[ast.ImportFrom(module="__future__",names=[ast.alias(name="annotations")],level=0),convert,command],type_ignores=[])
    ast.fix_missing_locations(module)
    exec(compile(module,"<offline-official-TUI-serializer>","exec"),scope)
    service=types.SimpleNamespace(_impl=types.SimpleNamespace(execute_command=lambda value:requests.append(value)))
    return lambda path,*args:scope["execute_command"](service,path,*args),requests


def test_actual_start_parameter_is_full_absolute_forward_path_through_official_serializer(native_input):
    root,slot,path,pipe,_,solver,_,calls,_=native_input
    execute,requests=official_fake_service()
    captured=[]
    def start(value):
        assert not pipe.closed
        assert "\\" not in value and not value.startswith('"') and not value.endswith('"')
        assert Path(value).is_absolute() and value==(path/"native.transcript.txt").as_posix()
        captured.append(value)
        execute("/file/start-transcript",value)
        Path(value).write_text("OFFLINE transcript fixture",encoding="utf-8")
    solver.tui.file.start_transcript=start
    report=w.native_collect(path,stage_root=root)
    assert len(calls)==len(captured)==len(requests)==1
    sent=requests[0].args.fields["tui_args"].list_value.values[0].string_value
    assert sent==captured[0] and sent.endswith("/native.transcript.txt")
    assert report["transcript_calls"]["start"]["stage"]=="completed"
    assert report["transcript_calls"]["stop"]["stage"]=="completed"
    for action in ("start","stop"):
        assert legacy.read_json(path/("transcript_"+action+"_attempted.json"))["stage"]=="attempted"
        assert legacy.read_json(path/("transcript_"+action+"_completed.json"))["stage"]=="completed"
        assert not (path/("transcript_"+action+"_failed.json")).exists()


def test_serializer_does_not_implicitly_quote_normalize_or_truncate_synthetic_path():
    execute,requests=official_fake_service()
    raw=r"D:\offline\new_case\native.transcript.txt"
    forward="D:/offline/new_case/native.transcript.txt"
    for value in (raw,forward): execute("/file/start-transcript",value)
    values=[r.args.fields["tui_args"].list_value.values[0].string_value for r in requests]
    assert values==[raw,forward]
    assert "\\n" in values[0] and "\n" not in values[0]
    assert "\\" not in values[1] and values[1]==forward


@pytest.mark.parametrize("existing",["file","directory"])
def test_existing_fixed_target_is_rejected_before_native_start_and_never_overwritten(native_input,existing):
    root,_,path,_,_,_,_,calls,connects=native_input
    target=path/"native.transcript.txt"
    if existing=="file":target.write_bytes(b"ORIGINAL OFFLINE EVIDENCE")
    else:target.mkdir()
    result=w.native_collect(path,stage_root=root)
    assert calls==connects==[]
    assert result["launch_uncertain"] is True and "native_adapter" in result["errors"]
    if existing=="file":assert target.read_bytes()==b"ORIGINAL OFFLINE EVIDENCE"
    else:assert target.is_dir()


def test_foreign_attempt_rejected_without_any_TUI_call(batch):
    root,slot,_=batch
    own=slot.safe(slot.ATTEMPT); own.mkdir(parents=True)
    foreign=root/"runs/foreign"; foreign.mkdir(parents=True)
    with pytest.raises(w.RuntimeGateError,match="exact fixed attempt"):
        w.fixed_transcript_target(slot,foreign)
    assert not (foreign/"native.transcript.txt").exists()


def test_reparse_fixed_target_rejected_without_native_call(batch,monkeypatch):
    root,slot,popen=batch
    attempt=slot.safe(slot.ATTEMPT); attempt.mkdir(parents=True)
    target=attempt/"native.transcript.txt"
    real=Path.lstat
    def lstat(path,*args,**kwargs):
        if path==target:return types.SimpleNamespace(st_mode=stat.S_IFREG,st_file_attributes=0x400)
        return real(path,*args,**kwargs)
    monkeypatch.setattr(Path,"lstat",lstat)
    with pytest.raises(legacy.BridgeError,match="reparse"):
        w.fixed_transcript_target(slot,attempt)
    popen.assert_not_called()


@pytest.mark.parametrize("suffix",["space root","quote\"root","semi;root","line\nroot","paren(root)"])
def test_path_requiring_unproven_quoting_is_refused_offline(tmp_path,suffix):
    # Windows cannot create quote/newline-containing names; fake safe path is
    # enough to test the lexical boundary, without filesystem mutation there.
    root=tmp_path/suffix
    attempt=root/"runs/fixed"
    slot=types.SimpleNamespace(root=root,ATTEMPT="runs/fixed",safe=lambda relative:root/relative)
    with mock.patch.object(w,"checked_path",return_value=attempt):
        with pytest.raises(w.RuntimeGateError,match="safe absolute forward-slash"):
            w.fixed_transcript_target(slot,attempt)


def test_start_failure_records_safe_phase_and_never_reads_mesh(native_input,monkeypatch):
    root,_,old_path,pipe,_,solver,_,calls,_=native_input
    slot=b.Batch002Slot("background",root)
    path,_=reserve(root,slot)
    parent=os.getppid()
    dump(path/"launcher.json",{"job_id":slot.JOB_ID,"pid":parent,"creation_time":123.5})
    dump(root/".ansys-task.lock",{"pid":parent,"attempt":str(path)})
    monkeypatch.setattr(b.Batch002Slot,"connection_gate",lambda self:{"status":"passed"})
    def fail(value):
        assert not pipe.closed and "\\" not in value
        assert value==(path/"native.transcript.txt").as_posix()
        assert legacy.read_json(path/"transcript_start_attempted.json")["stage"]=="attempted"
        raise SafeFixtureRpcError("OFFLINE-SECRET; synthetic sopen error detail")
    solver.tui.file.start_transcript=fail
    collector=mock.Mock(); monkeypatch.setattr(w,"collect_mesh",collector)
    report=w.native_collect(path,stage_root=root)
    assert len(calls)==1 and pipe.closed
    collector.assert_not_called()
    assert report["transcript_calls"]["start"]["stage"]=="failed" and "stop" not in report["transcript_calls"]
    saved=legacy.read_json(path/"transcript_start_failed.json")
    assert saved["safe_error"]["rpc_status_codes"]==["UNKNOWN"]
    assert "OFFLINE-SECRET" not in json.dumps(report) and "OFFLINE-SECRET" not in json.dumps(saved)
    assert not (path/"transcript_start_completed.json").exists()


def test_stop_failure_preserves_start_completion_and_error_without_success_claim(native_input):
    root,_,path,pipe,_,solver,_,calls,_=native_input
    def fail():
        assert not pipe.closed
        assert legacy.read_json(path/"transcript_stop_attempted.json")["stage"]=="attempted"
        raise SafeFixtureRpcError("OFFLINE-SECRET stop detail")
    solver.tui.file.stop_transcript=fail
    report=w.native_collect(path,stage_root=root)
    assert len(calls)==1 and pipe.closed
    assert report["transcript_calls"]["start"]["stage"]=="completed"
    assert report["transcript_calls"]["stop"]["stage"]=="failed"
    assert (path/"native.transcript.txt").exists() and "transcript_sha256" not in report
    assert "native_adapter" in report["errors"] and "OFFLINE-SECRET" not in json.dumps(report)
    assert legacy.read_json(path/"transcript_stop_failed.json")["stage"]=="failed"
    assert not (path/"transcript_stop_completed.json").exists()


def test_stop_without_completed_start_and_unknown_action_do_not_call_TUI(batch):
    root,slot,_=batch
    path=slot.safe(slot.ATTEMPT); path.mkdir(parents=True)
    tui=mock.Mock()
    with pytest.raises(w.RuntimeGateError):w.transcript_call(tui,"stop",slot,path,{})
    tui.tui.file.stop_transcript.assert_not_called()
    with pytest.raises(w.RuntimeGateError):w.transcript_call(tui,"arbitrary-user-command",slot,path,{})
    tui.tui.file.start_transcript.assert_not_called()


def test_recorder_native_APIs_are_exact_original_TUI_pair_and_safe_local_checkpoints():
    source=(STAGE/"inspection_batch002.py").read_text(encoding="utf-8")
    tree=ast.parse(source)
    recorder=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=="transcript_call")
    native=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=="native_collect")
    calls={ast.unparse(n.func) for n in ast.walk(recorder) if isinstance(n,ast.Call)}
    assert {name for name in calls if name.startswith("solver.")}=={"solver.tui.file.start_transcript","solver.tui.file.stop_transcript"}
    native_calls=[n for n in ast.walk(native) if isinstance(n,ast.Call)]
    assert sum(ast.unparse(n.func)=="subprocess.Popen" for n in native_calls)==1
    assert sum(ast.unparse(n.func)=="connect_ready" for n in native_calls)==1
    assert sum(ast.unparse(n.func)=="transcript_call" for n in native_calls)==2
    assert "solver.transcript.start" not in source and "solver.transcript.stop" not in source
    assert "start_transcript(str(transcript))" not in source
    for bad in ("scheme_eval", "exec", "eval", "initialize", "iterate", "write_mesh"):
        assert not any(name.endswith("."+bad) or name==bad for name in calls)


def test_protected_native_connection_PIPE_quality_and_cleanup_symbols_unchanged():
    source=(STAGE/"inspection_batch002.py").read_text(encoding="utf-8")
    expected={"NativeInputLease":"e082fd88fa2177cc9e4ba6e1f16d60c31cdc03c651c46e6efd1f6e08c979ac21",
              "collect_mesh":"d8e107eee19a8380dc8c7d4735101a270edf3d105f43b30a6d4cb4cdddb9fa95"}
    actual={n.name:hashlib.sha256(ast.get_source_segment(source,n).encode()).hexdigest() for n in ast.parse(source).body
            if isinstance(n,(ast.FunctionDef,ast.ClassDef)) and n.name in expected}
    assert actual==expected
    assert "cleanup_on_exit=normal_exit" in (STAGE/"inspection_batch002_connection.py").read_text(encoding="utf-8")


def test_current_zero_allowance_retires_stdin_validation_even_without_directory(batch,monkeypatch):
    root,_,popen=batch
    monkeypatch.setitem(b.POLICY,"stdin_validation_retired",True)
    slot=b.StdinValidationSlot(root)
    with pytest.raises(legacy.BridgeError,match="retired"):slot.start(b.STDIN_VALIDATION_START)
    assert not slot.safe(slot.ATTEMPT).exists() and slot.status()["remaining_attempts"]==0
    slot.safe(slot.ATTEMPT).mkdir(parents=True)
    with pytest.raises(w.RuntimeGateError,match="retired"):w.native_collect(slot.safe(slot.ATTEMPT),stage_root=root)
    assert b.POLICY["additional_attempts_authorized"]==b.POLICY["additional_connection_attempts"]==1
    assert b.POLICY["additional_QA_attempts"]==0
    popen.assert_not_called()


def test_historical_failed_record_cannot_become_passed_or_open_QA_after_recorder_repair(batch):
    root,_,popen=batch
    path=completed_connection(root)
    final=legacy.read_json(path/"probe_result.json")
    final["status"]="failed"
    final["native"]["errors"]={"native_adapter":{"rpc_status_codes":["UNKNOWN"]}}
    dump(path/"probe_result.json",final)
    before=(path/"probe_result.json").read_bytes()
    slot=b.FullRecordValidationSlot(root)
    answer=slot.start(b.FULL_RECORD_START)
    assert answer["job"]["status"]=="failed" and answer["job"]["connection_gate"]["status"]=="unverified"
    for name in ("background","arm"):
        qa=b.Batch002Slot(name,root)
        with pytest.raises(legacy.BridgeError):qa.start(b.START)
        assert not qa.safe(qa.ATTEMPT).exists()
    assert (path/"probe_result.json").read_bytes()==before
    popen.assert_not_called()
