#!/usr/bin/env python3
"""Create a report fragment from complete, directly timed experiments."""
import argparse
import csv
import html
import json
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument('session', type=Path)
args = parser.parse_args()
session = args.session
with (session / 'optimization_timing_summary.csv').open(newline='') as f:
    values = {r['mode']: r for r in csv.DictReader(f) if r['robot'] == 'all' and r['repeat'] == '1'}
if set(values) != {'baseline', 'delay', 'loss'}:
    raise ValueError('Need one complete timing run for each of baseline, delay and loss')
with (session / 'input_completion_summary.csv').open(newline='') as f:
    inputs = {r['mode']: r for r in csv.DictReader(f) if r['robot'] == 'all' and r['repeat'] == '1'}
with (session / 'metrics_provisional.csv').open(newline='') as f:
    trajectory_metrics = {r['mode']: r for r in csv.DictReader(f) if r['repeat'] == '1'}
labels = {'baseline': 'Без помех', 'delay': 'Задержка 100 мс', 'loss': 'Потери 1%'}


def fmt(value):
    return ('%.2f' % float(value)).replace('.', ',')


lines = []
for mode in ('baseline', 'delay', 'loss'):
    r = values[mode]
    data = json.loads((session / mode / 'run_1' / 'observed_stability.json').read_text())
    stable = data.get('wall_time_after_play_to_local_and_tf_stable_s')
    stable_label = fmt(stable) if stable is not None else 'не выявлена'
    ate = ('%.3f' % float(trajectory_metrics[mode]['ate_rmse_combined_m'])).replace('.', ',')
    lines.append('<tr><td>%s</td><td>%s</td><td>%s</td><td>%s</td><td>%s</td></tr>' %
                 (html.escape(labels[mode]), ate, fmt(r['mean_iteration_ms']) + ' / ' + fmt(r['p95_iteration_ms']),
                  fmt(r['sum_iteration_wall_s']), stable_label))
if set(inputs) != set(values) or set(trajectory_metrics) != set(values):
    raise ValueError('Missing input or ATE result for one or more modes')
input_times = ' / '.join(fmt(inputs[m]['last_pair_logged_after_play_start_s']) for m in ('baseline', 'delay', 'loss'))
snippet = '''<h2>Дополнительный прямой замер оптимизации</h2>
<p>Проведены новые полные проходы по одному на режим с записью длительности каждой итерации глобальной оптимизации и с записью TF. ATE пересчитана для этих же проходов методом из раздела выше. В таблице среднее и 95-й процентиль указаны по итерациям трёх роботов за время воспроизведения; сумма — сумма настенных длительностей итераций трёх параллельных потоков. Последний столбец — время после окончания воспроизведения до подтверждения 10 секунд без изменения локальных ключевых точек на 5 см и систем карт на 5 см / 0,5°. Метрики и исходные файлы сохранены в отдельной папке эксперимента.</p>
<table><tr><th>Режим</th><th>ATE, м</th><th>Среднее / P95, мс</th><th>Сумма, с</th><th>Устойчивость, с</th></tr>''' + ''.join(lines) + '''</table>
<p>Время регистрации последней ожидаемой пары облако–одометрия для всех роботов от начала запуска составило ''' + input_times + ''' с соответственно. Последнее облако в самом bag расположено примерно на 857-й секунде: далее в записи ещё идут сообщения одометрии. Метка журнала фиксируется перед добавлением пары в дальнейшую обработку, поэтому не означает завершения оптимизации.</p>
<p>В журнале робота 1 зарегистрировано 911 пар при 912 облаках во входном bag во всех трёх проходах; в режиме задержки у робота 2 — 986 при 987 облаках. Причина расхождения не установлена, поэтому наличие последней пары не следует считать подтверждением обработки каждого входного сообщения.</p>
<p>Сумма параллельных потоков не является общим временем исполнения или процессорным временем; она включает ожидание блокировок и не включает обработчики ROS, построение дескрипторов, ICP, визуализацию и ожидание сети. Критерий устойчивости — только наблюдение маркеров и TF: состояние всех очередей и последующие изменения не проверяются, а 10 секунд являются выбранным минимальным интервалом наблюдения. Поэтому это не доказанное время завершения всей обработки или сходимости. Новые проходы выполнены с отключением отладочных PCD и сжатой записью маркеров; существующая проблема смешения межроботного, входного и служебного трафика остаётся. Повторов для статистики по-прежнему недостаточно.</p>'''
(session / 'timing_report_fragment.html').write_text(snippet, encoding='utf-8')
print('Wrote', session / 'timing_report_fragment.html')
