# Multi-Proxy: эксперименты с тремя роботами

Воспроизведение совместного построения карты на S3E College, сравнение качества
связи и исследование вклада межроботной оптимизации. Скрипты сохраняют настройки,
карты, траектории и журналы в `results/`; входные данные и результаты в Git не
включены.

## Подготовка

Нужны Linux, Docker с Compose и `NET_ADMIN`, Git, Python 3 с NumPy, SciPy и
Matplotlib. Понадобится достаточно свободного места: минимум несколько гигабайт
на один запуск. Три контейнера получают фиксированные имена `multi-proxy0/1/2`
и адреса `172.0.0.2–4`; проверьте, что они и подсеть `172.0.0.0/24` не заняты.

Из корня репозитория:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt

mkdir -p src datasets/S3E
git clone https://github.com/HITSZ-NRSL/Multi-Proxy.git src/multi_proxy
git -C src/multi_proxy checkout e9f9e3a3ab3fdc4f38bfd724566904b9948264d4
bash scripts/setup_workspace.sh
bash scripts/download_ground_truth.sh
```

Поместите подготовленный `S3E_College_robot_odom.bag` в `datasets/S3E/`.
Источник: [Multi-Robot Cloud Odometry Dataset](https://www.modelscope.cn/datasets/OpenMushroom/Multi_robot_cloud-odometry),
описание в [Multi-Proxy](https://github.com/HITSZ-NRSL/Multi-Proxy). SHA-256
использованного bag: `a2b8a34aef60dcdc539cfe53475c619a4882bc2f385c2e33713883642b5db577`.
Полная последовательность длится 877,626 с. Скрипт отдельно скачивает RTK
[S3E Campus Road 1](https://huggingface.co/datasets/PengYu-Team/S3E/tree/main/S3Ev1/S3E_Campus_Road_1)
и проверяет хеши файлов.

`setup_workspace.sh` применяет патч для замера времени и отключения
отладочных PCD и устанавливает экспериментальные launch-файлы. Он требует
указанный коммит и отказывается перезаписывать изменённые launch-файлы.
После этого соберите образ и ROS-пакет, затем запустите master внутри robot 0:

```bash
docker compose build
docker compose up -d  # Для повторения с RViz используйте вариант из следующего раздела.
docker exec multi-proxy0 bash -c 'source /opt/ros/noetic/setup.bash; cd /ws_code; catkin_make -j2'
docker exec -d multi-proxy0 bash -c 'source /opt/ros/noetic/setup.bash; exec roscore'
```

Образ можно также предоставить заранее под именем `sylaranh/multi-proxy:latest`.
В первоначальных опытах использовался образ с ID
`sha256:04190b8a8cf3e068b69191a8dc103dbe1a2f63ae0be8d162aed19be65353c77b`;
сборка из Dockerfile может отличаться из-за версий внешних зависимостей.

## RViz

В измеренной серии RViz работал в контейнере robot 0 и получал облака всех
трёх роботов. Чтобы повторить эту нагрузку на Linux с локальным X11, перед
созданием контейнеров подготовьте полномочия для дисплея:

```bash
export MULTIPROXY_RVIZ_XAUTHORITY="$(mktemp)"
xauth nlist "$DISPLAY" | sed 's/^..../ffff/' | xauth -f "$MULTIPROXY_RVIZ_XAUTHORITY" nmerge -
test -s "$MULTIPROXY_RVIZ_XAUTHORITY"
docker compose -f compose.yaml -f docker/compose.rviz.yaml up -d
```

Соберите пакет и запустите ROS Master как выше. Затем запустите RViz:

```bash
docker exec -d multi-proxy0 bash -c 'source /ws_code/devel/setup.bash; exec rosrun rviz rviz -d /ws_code/src/multi_proxy/config/rviz.rviz __name:=multi_proxy_rviz'
docker exec multi-proxy0 bash -c 'source /ws_code/devel/setup.bash; rosnode ping -c 1 /multi_proxy_rviz'
```

Держите RViz открытым на протяжении серии. Для серии без графического
дисплея используйте основной Compose. После остановки RViz и удаления
контейнера можно удалить временный файл Xauthority.

## Эксперименты

**Качество связи и время оптимизации.** Один проход каждого режима:

```bash
bash experiments/part2/run_timing.sh
```

Режимы: без помех, задержка 100 мс на отправителе и потери 1% на отправителе.
Для трёх повторов каждого режима задайте `REPEATS=3`. Проход воспроизводит
полный bag в реальном времени, ждёт 120 с и сохраняет данные. Сценарий останавливает
текущие узлы роботов и временно меняет `/use_sim_time`; не запускайте его
параллельно с другими опытами. Результат — новая папка `results/part2/timing_*`.
Для короткой проверки: `MODES=baseline BAG_DURATION=30 SETTLE_SECONDS=15 bash experiments/part2/run_timing.sh`.

После запуска подставьте имя созданной папки:

```bash
SESSION=results/part2/timing_ИМЯ_ЗАПУСКА
python3 experiments/part2/analyze_metrics.py --session "$SESSION"
python3 experiments/part2/make_figures.py --session "$SESSION" --repeat 1
python3 experiments/part2/analyze_optimization_timing.py "$SESSION"
```

ATE рассчитывается с одним общим SE(3) для всех роботов каждого повтора;
соответствие robot 0/1/2 → Alpha/Bob/Carol определяется по первому baseline
и требует независимого подтверждения. Для расчёта времени последней входной
пары сначала сформируйте справку о входном bag:

```bash
mkdir -p results/part2
docker exec -i multi-proxy0 bash -c 'source /opt/ros/noetic/setup.bash; python3 - /datasets/S3E_College_robot_odom.bag' \
  < experiments/part2/inspect_input_end.py > results/part2/college_input_last_stamps.json
python3 experiments/part2/analyze_input_completion.py "$SESSION"
```

Чтобы объединить совместимые полноразмерные прогоны из нескольких сессий,
укажите сессии в хронологическом порядке. Скрипт проверяет хеши входа и
бинарного файла, параметры запуска, сетевые настройки и список узлов ROS. Держите состояние
RViz одинаковым на протяжении всех сравниваемых прогонов. Он пересчитывает
метрики для каждого повтора и сохраняет среднее, выборочное стандартное
отклонение, минимум и максимум по режимам. Сначала создайте справку о входном
bag по команде выше.

```bash
COMPARISON=results/part2/comparison
python3 experiments/part2/compare_repeats.py --out "$COMPARISON" "$SESSION"
python3 experiments/part2/make_figures.py --session "$COMPARISON" --repeat 1
```

**Вклад совместной оптимизации и обрыв данных.** Отдельные полные проходы:

```bash
bash experiments/part3/run_isolated.sh
bash experiments/part3/run_interruption.sh
```

В первом режиме heartbeat разделён по роботам, поэтому соседи не обнаруживаются.
Во втором сообщения четырёх типов проходят через ретранслятор, который блокирует
их на 120 с с 400-й секунды; heartbeat остаётся работающим. Перед сравнением
извлеките входную одометрию:

```bash
mkdir -p results/part3/input_odom
docker run --rm --network none -i -v "$PWD/datasets/S3E:/datasets:ro" \
  -v "$PWD/results/part3/input_odom:/out" --entrypoint /bin/bash sylaranh/multi-proxy:latest \
  -c 'source /opt/ros/noetic/setup.bash; python3 - /datasets/S3E_College_robot_odom.bag /out' \
  < experiments/part3/extract_input_odometry.py

ISO=results/part3/ИМЯ_ИЗОЛИРОВАННОГО_ЗАПУСКА
python3 experiments/part3/analyze_comparison.py "$ISO/isolated/run_1/log" \
  --joint-log "$SESSION/baseline/run_1/log" --out "$ISO/comparison"
```

Для успешного опыта с обрывом используйте путь к его `interruption/run_1`:

```bash
RUN=results/part3/ИМЯ_ЗАПУСКА/interruption/run_1
docker run --rm --network none -i -v "$PWD/$RUN:/data" \
  --entrypoint /bin/bash sylaranh/multi-proxy:latest \
  -c 'source /opt/ros/noetic/setup.bash; python3 - /data/markers.bag /data/markers.bag /data' \
  < experiments/part3/extract_observations.py
python3 experiments/part3/analyze_interruption.py "$RUN"
python3 experiments/part3/analyze_snapshots.py "$RUN"
python3 experiments/part3/analyze_comparison.py "$RUN/log" --candidate-shared-frame \
  --joint-log "$SESSION/baseline/run_1/log" --out "$RUN/comparison"
```

## Ограничения

- Сетевые фильтры заданы по IP контейнеров и затрагивают также часть входного,
  служебного и визуализационного трафика. Счётчики `tc` — не чистый трафик
  Multi-Proxy; журналы приложения не включают все накладные расходы и повторы TCP.
- Замеряется длительность итераций глобального оптимизатора. Сумма трёх
  параллельных потоков не равна общему времени исполнения или CPU time. Тишина
  маркеров и TF не доказывает опустошение очередей и сходимость.
- RTK–LiDAR смещение не проверено. В части 3 для отдельных локальных систем
  используется фиксированная начальная привязка к RTK по первым 100 с; расстояния
  между облаками характеризуют согласованность, а не абсолютную точность.
- Три повтора позволяют оценить разброс на данном стенде. Подготовленная
  одометрия использует Co-LRIO; независимость её исходного получения не подтверждена.

Исходники: [Multi-Proxy](https://github.com/HITSZ-NRSL/Multi-Proxy),
[S3E](https://pengyu-team.github.io/S3E/), [Co-LRIO](https://github.com/zhongshp/Co-LRIO).
