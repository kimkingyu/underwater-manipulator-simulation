"""Executable checks: synthetic labels are unit-test fixtures ONLY, never CFD replacements."""
import copy
import importlib.util
import json
from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from model_components import (BODY_ORDER, body_jacobians, build_template, component_features,
    frozen87_steady, link_kinematics, predict_split_hydro, recompose, save_json, sha256_file)
from fit_candidates import (compare_acceptance, direction_family, fit_once, rank_diagnostic,
    split_groups, training_scales)


class ComponentTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.contract = json.loads((ROOT/"contract.json").read_text(encoding="utf-8"))
        # These bboxes are explicitly synthetic and do not enter fitted candidates.
        cls.bodies = [{"id": b,"reference_CAD0_m": [0,0,0],
            "bbox_local_min_m": [-.15,-.12,-.1],"bbox_local_max_m": [.19,.13,.14],"walls": []}
            for b in BODY_ORDER]
        cls.defaults = [ROOT.parent/"11_theta_fit/matlab/theta_fit.json",ROOT.parent/"11_theta_fit/matlab/theta_fit.mat"]
        cls.before = {str(p):sha256_file(p) for p in cls.defaults}
        cls.rng = np.random.default_rng(1429)

    def model(self, candidate):
        m=build_template(self.contract,self.bodies,candidate)
        m["theta"]=[.4+(i%7)*.13 if p["kind"] not in ("even_transverse","even_pair") else (-1)**i*.3
                    for i,p in enumerate(m["parameters"])]
        return m

    def test_parameter_schema_units_and_dictionary_size(self):
        for candidate,size in (("A",45),("B",60),("C",90),("B_pair",66)):
            m=self.model(candidate)
            self.assertEqual(len(m["theta"]),size)
            self.assertEqual(len({p["name"] for p in m["parameters"]}),size)
            self.assertTrue(all(p["unit"]=="kg/m" for p in m["parameters"]))
            for p in m["parameters"]:
                b=self.bodies[p["body_index"]]
                self.assertTrue(np.all(np.asarray(p["point_m"])>=b["bbox_local_min_m"]))
                self.assertTrue(np.all(np.asarray(p["point_m"])<=b["bbox_local_max_m"]))

    def test_zero_speed_all_candidates(self):
        for candidate in ("A","B","C","B_pair"):
            out=predict_split_hydro(self.model(candidate),[.2,-.3,.7],np.zeros(6))
            np.testing.assert_array_equal(out["generalized"],np.zeros(9))

    def test_A_odd_and_B_even_extension(self):
        q=np.array([.3,-.2,.7]); v=np.array([.2,-.7,.5,0,0,0])
        a=self.model("A")
        plus=predict_split_hydro(a,q,v)["generalized"]
        minus=predict_split_hydro(a,q,-v)["generalized"]
        np.testing.assert_allclose(plus,-minus,atol=1e-12)
        b=self.model("B")
        b["theta"]=[t if p["kind"]=="even_transverse" else 0. for t,p in zip(b["theta"],b["parameters"])]
        plus=predict_split_hydro(b,q,v)["generalized"]
        minus=predict_split_hydro(b,q,-v)["generalized"]
        np.testing.assert_allclose(plus,minus,atol=1e-12)
        self.assertGreater(np.linalg.norm(plus),1e-4)
        self.assertLess(abs(np.r_[v,np.zeros(3)]@plus),1e-12)

    def test_B_pair_generates_even_moment_without_translation_force(self):
        model=self.model("B_pair")
        model["theta"]=[t if p["kind"]=="even_pair" else 0. for t,p in zip(model["theta"],model["parameters"])]
        q=np.array([.2,-.3,.5]); nu=np.array([.4,.8,-.3,0,0,0])
        plus=predict_split_hydro(model,q,nu)
        minus=predict_split_hydro(model,q,-nu)
        np.testing.assert_allclose(plus["links"][:,:3],0.,atol=1e-14)
        self.assertGreater(np.linalg.norm(plus["links"][:,3:]),1e-4)
        np.testing.assert_allclose(plus["generalized"],minus["generalized"],atol=1e-13)
        for _ in range(40):
            z=self.rng.normal(size=9);twists=body_jacobians(q,self.contract)@z
            for p in model["parameters"]:
                if p["kind"]!="even_pair":continue
                r1,r2=np.asarray(p["point_m"]),np.asarray(p["other_point_m"])
                n=np.asarray(p["direction"]);V=twists[p["body_index"]]
                v1=V[:3]+np.cross(V[3:],r1);v2=V[:3]+np.cross(V[3:],r2)
                f1=np.cross(v1,np.cross(n,v1));f2=-np.cross(v2,np.cross(n,v2))
                self.assertLess(abs(v1@f1),1e-12)
                self.assertLess(abs(v2@f2),1e-12)
            out=predict_split_hydro(model,q,z[:6],qdot=z[6:],allow_unvalidated=True)
            self.assertLess(abs(z@out["generalized"]),1e-11)

    def test_new_configuration_is_not_automatically_range_extrapolation(self):
        model=self.model("B")
        model["scope"].update({"observed_q":[[0,0,0],[.4,.8,-.6]],"max_training_speed_m_s":.8})
        flags=predict_split_hydro(model,[.2,.4,-.3],[.3,0,0,0,0,0])["scope_flags"]
        self.assertTrue(flags["new_configuration"])
        self.assertTrue(flags["out_of_sample_configuration"])
        self.assertTrue(flags["within_training_speed"])
        self.assertFalse(flags["outside_training_speed"])
        self.assertNotIn("extrapolation",flags)
        self.assertIn("not necessarily",flags["configuration_range_status"])

    def test_C_is_one_sided_and_passive(self):
        c=self.model("C")
        c["theta"]=[0.]*90
        c["theta"][0]=1.
        plus=predict_split_hydro(c,[0,0,0],[1,0,0,0,0,0])["hull"]
        minus=predict_split_hydro(c,[0,0,0],[-1,0,0,0,0,0])["hull"]
        self.assertLess(plus[0],0)
        np.testing.assert_array_equal(minus,np.zeros(6))

    def test_random_power_and_force_moment_closure(self):
        for _ in range(80):
            q=self.rng.uniform(-1,1,3)
            z=self.rng.normal(size=9)
            w=self.rng.normal(size=(5,6))
            kin=link_kinematics(q,self.contract)
            out=recompose(q,w,self.contract)
            manual=w[0]+w[1]
            tau=np.zeros(3)
            for i in range(3):
                f=kin["R"][i]@w[i+2,:3]
                m=kin["R"][i]@w[i+2,3:]
                manual=manual+np.r_[f,m+np.cross(kin["P"][i],f)]
                for j in range(i+1):
                    tau[j]+=kin["joint_axes"][j]@(m+np.cross(kin["P"][i]-kin["joint_positions"][j],f))
            np.testing.assert_allclose(out["total"],manual,atol=2e-14)
            np.testing.assert_allclose(out["tau_hydro"],tau,atol=2e-14)
            J=body_jacobians(q,self.contract)
            self.assertAlmostEqual(float(z@out["generalized"]),float(np.sum((J@z)*w)),places=12)

    def test_random_port_dissipation_and_zero_power_even_terms(self):
        for candidate in ("A","B","C","B_pair"):
            model=self.model(candidate)
            for _ in range(70):
                q=self.rng.uniform(-1.2,1.2,3); z=self.rng.normal(size=9)
                out=predict_split_hydro(model,q,z[:6],qdot=z[6:],allow_unvalidated=True)
                self.assertLessEqual(float(z@out["generalized"]),1e-10)
                self.assertFalse(out["scope_flags"]["supported_static_kinematics"])
                feature=component_features(model,q,z[:6],z[6:])
                twist=body_jacobians(q,self.contract)@z
                for k,p in enumerate(model["parameters"]):
                    work=float(twist[p["body_index"]]@feature[p["body_index"],:,k])
                    if p["kind"] in ("even_transverse","even_pair"):
                        self.assertLess(abs(work),1e-10)
                    else:
                        self.assertLessEqual(work,1e-10)

    def test_scope_rejection_and_flags(self):
        model=self.model("B")
        for args in ({"nu":[0,0,0,.1,0,0]}, {"nu":np.zeros(6),"qdot":[0,.1,0]},
                     {"nu":np.zeros(6),"nudot":[0,0,.1,0,0,0]}, {"nu":np.zeros(6),"steady":False}):
            with self.assertRaises(ValueError):
                predict_split_hydro(model,[0,0,0],**args)
            out=predict_split_hydro(model,[0,0,0],allow_unvalidated=True,**args)
            self.assertFalse(out["scope_flags"]["supported_static_kinematics"])
            self.assertFalse(out["scope_flags"]["inertial_model_included"])
            self.assertFalse(out["scope_flags"]["blind_validated"])

    def test_abnormal_inputs(self):
        m=self.model("A")
        for q,v in (([0,0],[0]*6),([0,0,float("nan")],[0]*6),([0]*3,[0]*5),([0]*3,[0,0,float("inf"),0,0,0])):
            with self.assertRaises(ValueError):predict_split_hydro(m,q,v)
        bad=copy.deepcopy(m); bad["theta"][0]=-1
        with self.assertRaises(ValueError):predict_split_hydro(bad,[0]*3,[0]*6)
        bad=copy.deepcopy(m); bad["parameters"][0]["direction"]=[2,0,0]
        with self.assertRaises(ValueError):predict_split_hydro(bad,[0]*3,[0]*6)
        bad=copy.deepcopy(m); bad["schema_version"]="unknown"
        with self.assertRaises(ValueError):predict_split_hydro(bad,[0]*3,[0]*6)

    def test_direction_families_keep_signs_speeds_and_poses_together(self):
        records=[{"pose_id":p,"nu":[s,0,0,0,0,0]} for p in ("q0","qb") for s in (-.8,-.3,.3,.8)]
        self.assertEqual(len(set(split_groups(records,"direction"))),1)
        self.assertEqual(len(set(split_groups(records,"pose"))),2)
        self.assertEqual(direction_family({"nu":[1,1,0,0,0,0]}),direction_family({"nu":[-.3,-.3,0,0,0,0]}))

    def test_train_scaling_ignores_heldout_labels(self):
        target={"hull":np.array([[1,2,3,4,5,6],[2,3,4,5,6,7],[1e9]*6],float)}
        first=training_scales(target,np.array([0,1]))
        target["hull"][2]=1e20
        second=training_scales(target,np.array([0,1]))
        np.testing.assert_array_equal(first["hull"],second["hull"])

    def test_identifiable_synthetic_recovery(self):
        for candidate in ("A","B","C","B_pair"):
            truth=self.model(candidate)
            records=[]
            for i in range(32):
                q=self.rng.normal(size=3); v=np.r_[self.rng.normal(size=3),[0.,0.,0.]]
                w=component_features(truth,q,v)@np.asarray(truth["theta"])
                records.append({"id":f"SYNTHETIC_{i}","q":q.tolist(),"nu":v.tolist(),"wrenches":w.tolist()})
            features=np.asarray([component_features(truth,r["q"],r["nu"]) for r in records])
            fitted=fit_once(build_template(self.contract,self.bodies,candidate),features,records,np.arange(32),0.)
            self.assertTrue(fitted["fit"]["full_rank"])
            np.testing.assert_allclose(fitted["theta"],truth["theta"],atol=2e-8,rtol=2e-8)

    def test_rank_failure_is_explicit(self):
        diag=rank_diagnostic(np.array([[1.,0.],[2.,0.],[3.,0.]]),["visible","unseen"])
        self.assertFalse(diag["full_rank"])
        self.assertEqual(diag["effective_rank"],1)
        self.assertIn("unseen",diag["unidentifiable_parameters"])
        wide=rank_diagnostic(np.array([[1.,0.,0.]]),["one","two","three"])
        self.assertEqual(wide["nullity"],2)
        self.assertEqual(wide["unidentifiable_parameters"],["two","three"])

    def test_retention_scope_does_not_hide_rejected_candidate(self):
        model=self.model("C")
        model["retention_status"]="rejected_nonidentifiable_development_folds"
        model["development_cv_complete"]={"pose":True,"direction":False}
        flags=predict_split_hydro(model,[0,0,0],[.3,0,0,0,0,0])["scope_flags"]
        self.assertIn("rejected",flags["candidate_retention_status"])
        self.assertFalse(flags["complete_development_identifiability"])
        self.assertFalse(flags["full_fit_identified"])

    def test_saved_cv_integrity_and_actual_training_scales(self):
        path=ROOT/"reports/model_development.json"
        if not path.is_file():self.skipTest("development CV has not yet been run")
        data=json.loads((ROOT/"data/component_dataset.json").read_text(encoding="utf-8"))
        report=json.loads(path.read_text(encoding="utf-8"))
        row_map={r["id"]:r for r in data["steady"]}
        for population,result in report["populations"].items():
            all_ids={r["id"] for r in data["steady"] if population!="strict37" or r["strict"]}
            for c,cr in result["candidates"].items():
                for scheme,cv in cr["development_cv"].items():
                    tested=[]
                    for fold in cv["folds"]:
                        tr,te=set(fold["train_ids"]),set(fold["test_ids"])
                        self.assertFalse(tr&te)
                        self.assertEqual(tr|te,all_ids)
                        tested.extend(te)
                        for inner in fold["inner_selection"]["folds"]:
                            itr,ite=set(inner["train_ids"]),set(inner["test_ids"])
                            self.assertFalse(itr&ite)
                            self.assertEqual(itr|ite,tr)
                        tr_labels=set(split_groups([row_map[x] for x in tr],scheme))
                        te_labels=set(split_groups([row_map[x] for x in te],scheme))
                        self.assertFalse(tr_labels&te_labels)
                        for b,diag in enumerate(fold["component_diagnostics"]):
                            y=np.asarray([row_map[x]["wrenches"][b] for x in fold["train_ids"]])
                            scale=np.maximum(np.sqrt(np.mean(y*y,axis=0)),[1,1,1,.1,.1,.1])
                            np.testing.assert_allclose(diag["target_scale_train_only"],scale,atol=1e-13)
                    self.assertEqual(len(tested),len(all_ids))
                    self.assertEqual(set(tested),all_ids)

    def test_generalized9_report_has_nine_channels_and_SI_units(self):
        path=ROOT/"reports/model_generalized9_metrics.json"
        if not path.is_file():self.skipTest("nine-target supplementary view not yet exported")
        report=json.loads(path.read_text(encoding="utf-8"))
        self.assertEqual(report["order"],["Fx","Fy","Fz","Mx","My","Mz","J1","J2","J3"])
        for models in report["populations"].values():
            for schemes in models.values():
                for metric in schemes.values():
                    self.assertEqual(len(metric["channels"]),9)
                    self.assertEqual([c["unit"] for c in metric["channels"]],["N"]*3+["N*m"]*6)
        self.assertFalse(report["populations"]["strict37"]["C"]["direction"]["eligible_complete_score"])

    def test_b_pair_preserves_ABC_and_uses_identical_outer_folds(self):
        path=ROOT/"reports/model_b_pair_development.json"
        if not path.is_file():self.skipTest("bounded supplement has not yet been run")
        pair=json.loads(path.read_text(encoding="utf-8"))
        original=json.loads((ROOT/"reports/model_development.json").read_text(encoding="utf-8"))
        self.assertEqual(pair["original_ABC_hashes_before"],pair["original_ABC_hashes_after"])
        self.assertEqual(pair["original_ABC_hashes_after"],{p:sha256_file(p) for p in pair["original_ABC_hashes_after"]})
        for population,result in pair["populations"].items():
            for scheme,cv in result["development_cv"].items():
                old=original["populations"][population]["candidates"]["B"]["development_cv"][scheme]
                self.assertEqual(cv["prediction_ids"],old["prediction_ids"])
                self.assertEqual(len(cv["folds"]),len(old["folds"]))
                for fold,old_fold in zip(cv["folds"],old["folds"]):
                    self.assertEqual(fold["train_ids"],old_fold["train_ids"])
                    self.assertEqual(fold["test_ids"],old_fold["test_ids"])
                    outer=set(fold["train_ids"])
                    for inner in fold["inner_selection"]["folds"]:
                        self.assertFalse(set(inner["train_ids"])&set(inner["test_ids"]))
                        self.assertEqual(set(inner["train_ids"])|set(inner["test_ids"]),outer)
        audit=pair["component_passivity_assumption"]
        self.assertEqual(len(audit["positive_component_power_cases"]),7)
        self.assertLess(audit["total_power_range_W"][1],0)
        self.assertTrue(audit["all_cases_retained"])

    def test_frozen87_retains_even_added_coriolis(self):
        theta=np.zeros(87); theta[:3]=[1,2,3]
        out=frozen87_steady(theta,[0,0,0],[1,1,0,0,0,0],self.contract)
        self.assertNotIn("hull",out)
        self.assertEqual(out["total"][5],-1.)
        minus=frozen87_steady(theta,[0,0,0],[-1,-1,0,0,0,0],self.contract)
        np.testing.assert_allclose(out["total"],minus["total"])

    def test_shared_dataset_closes_if_present(self):
        path=ROOT/"data/component_dataset.json"
        if not path.is_file():self.skipTest("shared authorized dataset not yet available")
        data=json.loads(path.read_text(encoding="utf-8"))
        for r in data["steady"]:
            out=recompose(r["q"],r["wrenches"],self.contract)
            np.testing.assert_allclose(out["total"],r["total"],atol=1e-9)
            np.testing.assert_allclose(out["tau_hydro"],r["joint_tau"],atol=1e-9)

    def test_zz_default_theta_hash_unchanged(self):
        self.assertEqual(self.before,{str(p):sha256_file(p) for p in self.defaults})


def export_cross_language_fixture():
    contract=json.loads((ROOT/"contract.json").read_text(encoding="utf-8"))
    rng=np.random.default_rng(1429)
    synthetic=[{"id":b,"reference_CAD0_m":[0,0,0],"bbox_local_min_m":[-.15,-.12,-.1],
        "bbox_local_max_m":[.19,.13,.14],"walls":[]} for b in BODY_ORDER]
    cases=[]
    for candidate in ("A","B","C","B_pair"):
        model=build_template(contract,synthetic,candidate)
        model["theta"]=[float(rng.normal()) if p["kind"] in ("even_transverse","even_pair") else float(rng.uniform(.1,3)) for p in model["parameters"]]
        for i in range(6):
            q=rng.uniform(-1,1,3); z=rng.normal(size=9)
            if i<3:z[3:]=0
            options={"qdot":z[6:],"allow_unvalidated":True}
            out=predict_split_hydro(model,q,z[:6],qdot=z[6:],allow_unvalidated=True)
            cases.append({"model":model,"q":q,"nu":z[:6],"options":options,"expected":out,
                          "expected_J":body_jacobians(q,contract)[2:].tolist()})
    fitted_paths=[ROOT/"models"/f"candidate_{c}_strict37.json" for c in ("A","B","C","B_pair")]
    for path in fitted_paths:
        if path.is_file():
            model=json.loads(path.read_text(encoding="utf-8"))
            q=[.2,-.5,.3]; nu=[.8,-.4,.1,0,0,0]
            out=predict_split_hydro(model,q,nu)
            cases.append({"model":model,"q":q,"nu":nu,"options":{"qdot":[0,0,0],"allow_unvalidated":False},
                          "expected":out,"expected_J":body_jacobians(q,contract)[2:].tolist()})
    theta=json.loads((ROOT/"frozen/theta87_baseline.json").read_text(encoding="utf-8"))["theta"]
    old=[]
    for _ in range(20):
        q=rng.uniform(-1,1,3); nu=np.r_[rng.uniform(-1,1,3),np.zeros(3)]
        old.append({"q":q,"nu":nu,"expected":frozen87_steady(theta,q,nu,contract)["generalized"]})
    save_json(ROOT/"reports/model_cross_language_fixture.json",{"fixture_role":"synthetic unit tests plus labelled final development candidates; NOT CFD data",
        "cases":cases,"frozen87_cases":old,"theta87":theta,"default_theta_hashes":ComponentTests.before})


if __name__=="__main__":
    suite=unittest.defaultTestLoader.loadTestsFromTestCase(ComponentTests)
    result=unittest.TextTestRunner(verbosity=2).run(suite)
    save_json(ROOT/"reports/model_python_tests.json",{"tests_run":result.testsRun,"passed":result.wasSuccessful(),
        "failures":[str(test)+"\n"+message for test,message in result.failures],
        "errors":[str(test)+"\n"+message for test,message in result.errors],"skipped":result.skipped,
        "default_theta_hashes_unchanged":ComponentTests.before=={str(p):sha256_file(p) for p in ComponentTests.defaults}})
    if result.wasSuccessful():export_cross_language_fixture()
    sys.exit(0 if result.wasSuccessful() else 1)
