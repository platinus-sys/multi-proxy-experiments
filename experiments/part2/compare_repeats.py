#!/usr/bin/env python3
"""Compare complete, compatible timing trials from one or more sessions."""
import argparse
import csv
from html import escape
import json
from pathlib import Path
import statistics
import subprocess
import sys

HERE = Path(__file__).resolve().parent
MODES = {'baseline': 'Без помех', 'delay': 'Задержка', 'loss': 'Потери'}
METRICS = (
    ('play_wall_seconds', 'Воспроизв., с'),
    ('play_and_settle_wall_seconds', 'С ожиданием, с'),
    ('save_wall_seconds', 'Запись, с'),
    ('play_to_saved_wall_seconds', 'От старта bag до сохранения, с'),
    ('sum_iteration_wall_s', 'Оптимизация, потоки·с'),
    ('mean_iteration_ms', 'Средняя итерация, мс'),
    ('p95_iteration_ms', 'P95 итерации, мс'),
    ('netem_gb', 'Трафик tc, ГБ'),
    ('payload_mb', 'Публикации, МБ'),
    ('netem_dropped_packets', 'Отброшено пакетов'),
    ('ate_rmse_combined_m', 'ATE*, м'),
    ('last_pair_logged_after_play_start_s', 'Последняя пара, с'),
    ('stability_after_play_s', 'Устойчивость, с'),
)


def read_csv(path):
    with path.open(newline='') as f:
        return list(csv.DictReader(f))


def write_csv(path, rows):
    with path.open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def key(row):
    return row['mode'], row['repeat']


def protocol(session):
    lines = (session / 'environment.txt').read_text().splitlines()
    checks = [next(line for line in lines if line.startswith(prefix)) for prefix in (
        'settle_seconds=', 'bag_rate=', 'bag_duration=', 'optimization_timing=',
        'debug_pcd=', 'marker_compression=', 'master=')]
    for suffix in (' /ws_code/devel/lib/multi_proxy/multi_proxy_node',
                   ' datasets/S3E/S3E_College_robot_odom.bag'):
        checks.append(next(line for line in lines if line.endswith(suffix)))
    checks.extend(line for line in lines if ' image_id=' in line)
    checks.extend((session / name).read_bytes() for name in ('config.yaml', 'launch.xml', 'network.sh'))
    return checks


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('sessions', type=Path, nargs='+', help='Sessions in chronological order')
    args = parser.parse_args()
    sessions = [path.resolve() for path in args.sessions]
    if len(set(sessions)) != len(sessions):
        parser.error('Each session must appear only once')
    protocol_ref = protocol(sessions[0])
    if 'bag_duration=0' not in protocol_ref or 'optimization_timing=enabled' not in protocol_ref:
        parser.error('Full-length timing protocol required')
    manifest = []
    nodes_ref = None
    counts = dict.fromkeys(MODES, 0)
    for session in sessions:
        if protocol(session) != protocol_ref:
            parser.error(f'Incompatible protocol in {session}')
        for mode in MODES:
            runs = sorted((session / mode).glob('run_*'), key=lambda p: int(p.name[4:]))
            for run in runs:
                if not (run / 'COMPLETE.txt').is_file() or (run / 'FAILED.txt').exists():
                    continue
                nodes = set((run / 'rosnodes.txt').read_text().splitlines())
                if nodes_ref is None:
                    nodes_ref = nodes
                if nodes != nodes_ref:
                    parser.error(f'ROS nodes differ in {run}: {sorted(nodes ^ nodes_ref)}')
                counts[mode] += 1
                manifest.append({'mode': mode, 'repeat': counts[mode], 'source_run': str(run)})
    if not all(counts.values()):
        parser.error(f'Need a complete run for each mode, got {counts}')
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=False)
    write_csv(out / 'run_manifest.csv', manifest)
    for entry in manifest:
        run = Path(entry['source_run'])
        target = out / entry['mode'] / f"run_{entry['repeat']}"
        target.mkdir(parents=True)
        for name in ('COMPLETE.txt', 'log', 'tc_before.txt', 'tc_after.txt', 'observed_stability.json'):
            (target / name).symlink_to(run / name)
        meta = (run / 'metadata.txt').read_text().splitlines()
        if f"mode={entry['mode']}" not in meta or f'repeat={run.name[4:]}' not in meta:
            raise ValueError(f'Metadata does not match directory: {run}')
        meta = [f"repeat={entry['repeat']}" if line.startswith('repeat=') else line for line in meta]
        (target / 'metadata.txt').write_text('\n'.join(meta) + '\n')

    argument_lists = (
        ('summarize.py', out), ('analyze_optimization_timing.py', out),
        ('analyze_input_completion.py', out), ('analyze_metrics.py', '--session', out),
    )
    for script, *arguments in argument_lists:
        subprocess.run([sys.executable, HERE / script, *arguments], check=True)
    timing = {key(row): row for row in read_csv(out / 'optimization_timing_summary.csv') if row['robot'] == 'all'}
    inputs = {key(row): row for row in read_csv(out / 'input_completion_summary.csv') if row['robot'] == 'all'}
    rows = []
    for row in read_csv(out / 'metrics_provisional.csv'):
        mode, repeat = key(row)
        run = out / mode / f'run_{repeat}'
        stability = json.loads((run / 'observed_stability.json').read_text())
        rows.append({
            **row, **{k: timing[mode, repeat][k] for k in (
                'iterations', 'sum_iteration_wall_s', 'mean_iteration_ms', 'p95_iteration_ms')},
            'play_to_saved_wall_seconds': round(
                float(row['play_and_settle_wall_seconds']) + float(row['save_wall_seconds']), 3),
            'netem_gb': int(row['netem_bytes']) / 1e9,
            'payload_mb': int(row['published_logged_messages_bytes']) / 1e6,
            'last_pair_logged_after_play_start_s': inputs.get((mode, repeat), {}).get('last_pair_logged_after_play_start_s'),
            'stability_after_play_s': stability['wall_time_after_play_to_local_and_tf_stable_s'],
        })
    write_csv(out / 'comparison_per_run.csv', rows)

    aggregate = []
    for mode in MODES:
        subset = [row for row in rows if row['mode'] == mode]
        for field, _ in METRICS:
            values = [float(row[field]) for row in subset if row[field] not in (None, '')]
            aggregate.append({
                'mode': mode, 'metric': field, 'runs': len(subset), 'observations': len(values),
                'mean': statistics.mean(values) if values else None,
                'sample_sd': statistics.stdev(values) if len(values) > 1 else None,
                'min': min(values) if values else None, 'max': max(values) if values else None,
            })
    write_csv(out / 'comparison_by_mode.csv', aggregate)
    write_fragment(out, rows, aggregate, counts, '/multi_proxy_rviz' in nodes_ref)


def number(value):
    return '—' if value in (None, '') else f'{float(value):.3f}'.replace('.', ',')


def table(headers, rows):
    return '<table><tr>' + ''.join(f'<th>{escape(value)}</th>' for value in headers) + '</tr>' + \
        ''.join('<tr>' + ''.join(f'<td>{escape(value)}</td>' for value in row) + '</tr>' for row in rows) + '</table>'


def write_fragment(out, runs, aggregate, counts, rviz_running):
    grouped = {(row['mode'], row['metric']): row for row in aggregate}
    per_run = {(mode, field): [number(row[field]) for row in runs if row['mode'] == mode]
               for mode in MODES for field, _ in METRICS}
    report = '<h2>Сравнение повторов с прямым замером времени</h2>'
    report += '<p>Полный S3E College при скорости 1×, ожидание 120 с после воспроизведения. ' + \
        ', '.join(f'{label.lower()} — {counts[mode]}' for mode, label in MODES.items()) + \
        '. В каждой ячейке приведены значения по отдельным повторам и среднее ± выборочное стандартное отклонение.</p>'
    report += ('<p>RViz работал в каждом проходе с исходной конфигурацией визуализации.</p>' if rviz_running else
               '<p>Прогоны выполнены без RViz.</p>')
    rows = []
    for field, label in METRICS:
        cells = [label]
        for mode in MODES:
            group = grouped[mode, field]
            cells.append(' / '.join(per_run[mode, field]) + f" ({number(group['mean'])} ± {number(group['sample_sd'])})")
        rows.append(cells)
    report += table(['Показатель', *MODES.values()], rows)
    report += '<h3>Длительности этапов каждого прогона</h3>'
    report += '<p>Отсчёт начинается с запуска rosbag. Время сохранения включает проверку стабильного размера файлов.</p>'
    time_rows = []
    for row in runs:
        settling = float(row['play_and_settle_wall_seconds']) - float(row['play_wall_seconds'])
        time_rows.append([
            MODES[row['mode']] + ' ' + row['repeat'], number(row['play_wall_seconds']),
            number(settling), number(row['save_wall_seconds']), number(row['play_to_saved_wall_seconds']),
        ])
    report += table(['Режим и повтор', 'Bag, с', 'Ожидание, с', 'Сохранение, с', 'Итого, с'], time_rows)
    metrics = ('sum_iteration_wall_s', 'netem_gb', 'ate_rmse_combined_m')
    baseline = {field: grouped['baseline', field] for field in metrics}
    descriptions = []
    for mode in ('delay', 'loss'):
        changes = [100 * (grouped[mode, field]['mean'] / baseline[field]['mean'] - 1)
                   for field in metrics]
        percentages = [f"{value:+.1f}%".replace('.', ',') for value in changes]
        descriptions.append(f"{MODES[mode]} — " + ', '.join(percentages))
    report += ('<p>Изменения среднего относительно режима без помех приведены в порядке '
               '«сумма длительностей оптимизации, трафик tc, предварительная ATE». '
               + '. '.join(descriptions) + '.</p>')
    matched = []
    for robot in range(3):
        samples = [int(row[f'robot_{robot}_matched']) for row in runs]
        low, high = min(samples), max(samples)
        matched.append(str(low) if low == high else f'{low}–{high}')
    report += '<p>Число сопоставленных с RTK позиций по роботам 0/1/2 — ' + ' / '.join(matched) + '.</p>'
    report += """<p>Оптимизация измерялась по steady_clock внутри каждой полной итерации во время воспроизведения. Суммируются длительности трёх параллельных потоков, включая ожидание блокировок. Замер исключает ICP, дескрипторы и обработчики ROS. Время воспроизведения включает подачу данных с заданной скоростью, а время с ожиданием включает фиксированное окно 120 с.</p>
<p>Устойчивость измерена от конца воспроизведения до подтверждения 10 секунд без заметных изменений локальных ключевых кадров и TF при порогах 5 см и 0,5°. Прочерк означает отсутствие наблюдения, удовлетворяющего критерию. Состояние очередей и сходимость этим критерием не проверяются. Последняя пара облако–одометрия измерена от старта воспроизведения до записи пары в журнал перед дальнейшей обработкой.</p>
<p>Счётчики tc включают трафик к IP других роботов, в том числе часть входного потока и ROS-служебных сообщений. Публикации учитывают журналы приложения для дескрипторов, облаков, межроботных связей и dual-сообщений. Байтовые единицы десятичные.</p>
<p>* ATE предварительная. Сопоставление с RTK по ближайшей метке в пределах 0,55 с, одна общая привязка SE(3) на повтор. Соответствие robot 0/1/2 → Alpha/Bob/Carol выбрано по первому baseline. Внешняя калибровка RTK–LiDAR и соответствие роботов требуют независимой проверки. Повторы проводились последовательными сериями в порядке запуска. Разброс включает изменения нагрузки на хост между прогонами.</p>"""
    (out / 'timing_report_fragment.html').write_text(report, encoding='utf-8')
    print(f'Comparison: {out}')


if __name__ == '__main__':
    main()
