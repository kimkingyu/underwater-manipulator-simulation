"""Update mutable progress artifacts from execution metadata only, never from blind load values."""
import datetime
import html
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent


def main():
    cfg = json.loads((ROOT/'cfd/validation_cases.json').read_text(encoding='utf-8'))
    rows = []
    for case in cfg['cases']:
        attempts = sorted((ROOT/'cfd/runs'/case['id']).glob('attempt_*/result.json'))
        row = {'id':case['id'],'mode':case['mode'],'status':'not_started','outcome':None,'last_step':None,'iterations':None,'time_steps':None}
        if attempts:
            d = json.loads(attempts[-1].read_text(encoding='utf-8'))
            row.update({'status':d.get('status'),'outcome':d.get('outcome'),'attempt':attempts[-1].parent.name})
            row.update({k:d.get('terminal_state',{}).get(k) for k in ('iterations','time_steps')})
            steps=d.get('steps',[])
            if steps: row['last_step']=steps[-1].get('name',steps[-1].get('step'))
        rows.append(row)
    completed=sum(r['status']=='completed' for r in rows)
    strict=sum(r['outcome']=='strict_window_drift' for r in rows)
    capped=sum(r['outcome']=='capped_unsteady_mean' for r in rows)
    summary={'snapshot_local':datetime.datetime.now().astimezone().isoformat(),'new_cases_planned':len(rows),
             'new_cases_execution_completed':completed,'strict_steady_completed':strict,'capped_steady_completed':capped,
             'blind_scores_computed_here':False,'models_and_predictions_frozen':True,'default_theta_replaced':False,
             'python_tests_passed_before_cfd':99,'cases':rows}
    (ROOT/'reports/progress_snapshot.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
    table=''.join('<tr>'+''.join('<td>'+html.escape(str(r.get(k) if r.get(k) is not None else '—'))+'</td>' for k in ('id','status','outcome','iterations','time_steps','last_step'))+'</tr>' for r in rows)
    page='''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>阶段14执行进度</title><style>body{font:15px/1.7 Arial,"Microsoft YaHei",sans-serif;color:#20344c;background:#f3f6fa}main{max-width:1180px;margin:24px auto;background:white;padding:28px 36px}h1,h2{color:#17365d}table{border-collapse:collapse;width:100%;font-size:13px}th,td{border:1px solid #d5dfea;padding:8px;text-align:left}th{background:#17365d;color:white}.note{background:#fff1d9;padding:14px}a{color:#155faf}</style><main><h1>水动力模型：独立验证执行进度</h1><p>快照时间：'''+html.escape(summary['snapshot_local'])+'''</p><p>计划9例，执行完成 <b>'''+str(completed)+'''</b>；其中稳态达到载荷窗口判据 <b>'''+str(strict)+'''</b>，达到迭代上限 <b>'''+str(capped)+'''</b>。本页不读取或计算预测误差。</p><table><tr><th>工况</th><th>执行状态</th><th>实际结果标记</th><th>稳态迭代数</th><th>瞬态步数</th><th>最后步骤</th></tr>'''+table+'''</table><h2>已完成的前置工作</h2><ul><li>46工况分体数据/单位/坐标/功率闭合审计完成；运行前99项Python检查及旧MATLAB代数回归通过。</li><li>许可已由用户恢复；qe精确几何、水体、网格、base及真实报告API均通过。qe为326,927多面体，min orth=0.140964，0.01门槛未放宽。</li><li>8稳态+1振荡的输入、模型、事前预测、评分规则和关键代码已在真实CFD开始前锁定。</li><li>旧默认θ、源数据、已交付文件均保留；未自动提交或推送。</li></ul><div class="note"><b>执行完成不等于模型验证通过。</b>稳态到2500次上限不称收敛；全部9例完成后按冻结规则评分。当前稳态候选在开发阶段无优胜；动态模型的已有数据拟合改善也不能当独立验证。本轮不验证关节独立运动或任意频率/幅值。</div><h2>记录入口</h2><ul><li><a href="../frozen/blind_protocol.json">最终盲测协议及34项指纹</a></li><li><a href="../frozen/research_model_manifest.json">较早的研究冻结快照（其中waiting状态仅代表当时）</a></li><li><a href="data_audit.json">数据审计</a></li><li><a href="model_development.html">ABC开发报告</a> / <a href="model_b_pair_development.html">B_pair补充</a></li><li><a href="dynamic_development.html">动态开发报告</a></li><li><a href="component_power_audit.json">局部分体功率限制</a></li></ul></main></html>'''
    (ROOT/'reports/live_status.html').write_text(page,encoding='utf-8')
    print('Metadata-only progress:',completed,'/',len(rows),'execution complete;',strict,'strict steady;',capped,'capped steady')


if __name__=='__main__': main()
