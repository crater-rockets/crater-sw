from itertools import batched
from pathlib import Path
from typing import Callable, Iterator
import rerun as rr
import polars as pl
from rerun_bindings import Recording
import scipy as sp
import numpy as np
import matplotlib.pyplot as plt
import pyarrow as pa


class RerunReader:
    rec: Recording
    timeline: str
    entities: dict[str, str]

    def __init__(self, file: Path, timeline: str, entities: dict[str, str]):
        self.rec = rr.dataframe.load_recording(file)
        self.entities = entities
        self.timeline = timeline

    def iter_tables(self, num_rows: int, range_secs: tuple[float, float] | None = None) -> Iterator[pa.Table]:
        view = self.rec.view(
            index=self.timeline, contents={k: [v] for k, v in self.entities.items()}
        )
        if range is not None:
            view = view.filter_range_secs(range_secs[0], range_secs[1])
        reader = view.select(
            columns=[self.timeline] + [f"{k}:{v}" for k, v in self.entities.items()]
        )

        batch_list = []
        count_rows = 0
        for batch in reader:
            batch_list.append(batch)
            count_rows += batch.num_rows

            if count_rows >= num_rows:
                yield pa.Table.from_batches(batch_list[0:num_rows])
                batch_list = batch_list[num_rows:]
                count_rows = count_rows - num_rows

        # Last batch may not have num_rows elements available
        if len(batch_list) > 0:
            yield pa.Table.from_batches(batch_list)


class FrequencyDomainAverage:
    n: int
    sampling_freq: float
    values: np.ndarray
    niter: int
    window: np.ndarray
    rfreq_fn: Callable[[np.ndarray], np.ndarray]

    def __init__(
        self,
        n: int,
        sampling_freq: float,
        rfreq_fn: Callable[[np.ndarray], np.ndarray],
        window: np.ndarray,
    ):
        if len(window.shape) != 1:
            raise ValueError(f"window must be a 1D array, but has shape {window.shape}")

        self.sampling_freq = sampling_freq
        self.n = n
        self.values = np.zeros(n // 2 + 1)
        self.niter = 0
        self.rfreq_fn = rfreq_fn
        self.window = window

    def update(self, time_domain: np.ndarray):
        w = time_domain * self.window
        self.values += self.rfreq_fn(w, self.sampling_freq)
        self.niter += 1

    def get_freq_domain(self) -> tuple[np.ndarray, np.ndarray]:
        freq = sp.fft.rfftfreq(self.n, 1 / self.sampling_freq)
        return freq, self.values / self.niter


N = 2**14
MAXITER = 50000

fft_window = sp.signal.windows.get_window("boxcar", N)

filename = Path("day.rrd")
timeline = "rel_system_time"
entities = {
    "/sensors/icm42688/accel_m_s2/x": "Scalar",
    "/sensors/icm42688/accel_m_s2/y": "Scalar",
    "/sensors/icm42688/accel_m_s2/z": "Scalar",
    "/sensors/icm42688/ang_vel_deg_s/x": "Scalar",
    "/sensors/icm42688/ang_vel_deg_s/y": "Scalar",
    "/sensors/icm42688/ang_vel_deg_s/z": "Scalar",
}

print("Loading recording...")
reader = RerunReader(filename, timeline, entities)

def psd_fn(x: np.ndarray, f: float) -> np.ndarray:
    _, psd = sp.signal.periodogram(x, f)
    return psd


def fft_fn(x: np.ndarray, _: float) -> np.ndarray:
    return np.abs(2 * sp.fft.rfft(x) / len(x))


fft_avgs = {
    e: FrequencyDomainAverage(N, 200, fft_fn, fft_window) for e in entities.keys()
}
psd_avgs = {
    e: FrequencyDomainAverage(N, 200, psd_fn, fft_window) for e in entities.keys()
}

print("Starting data processing...")

for i, table in enumerate(reader.iter_tables(N, (3600, 3600*3))):
    if table.num_rows != N:
        break

    df = pl.from_arrow(table.rename_columns([timeline] + list(entities.keys())))

    df = df.with_columns(
        time=pl.col(timeline).dt.total_microseconds() / 1000000,
        dt=(pl.col(timeline).dt.total_microseconds() / 1000000).diff(),
    )
    df = df.with_columns([pl.col(e).list.first().alias(e) for e in entities])
    df = df.with_columns([(pl.col(e) - pl.col(e).mean()).alias(e) for e in entities])

    for e in entities.keys():
        fft_avgs[e].update(df[e])
        psd_avgs[e].update(df[e])

    print(f"{i+1} - Processed {(i+1) * N} rows, {timeline} = {df[timeline][-1]}...")

    if i > MAXITER:
        break

print("Plotting...")

fig, axes = plt.subplots(len(entities), 1, sharex=True)
if not isinstance(axes, np.ndarray):
    axes = np.array([axes])

for i, e in enumerate(entities.keys()):
    ax = axes[i]
    x, y = fft_avgs[e].get_freq_domain()

    ax.plot(x, y)
    ax.grid(which="both")
    ax.set_title(e)
plt.suptitle("FFT")


fig, axes = plt.subplots(len(entities), 1, sharex=True)
if not isinstance(axes, np.ndarray):
    axes = np.array([axes])

for i, e in enumerate(entities.keys()):
    ax = axes[i]
    x, y = psd_avgs[e].get_freq_domain()

    ax.semilogy(x, y)
    ax.grid(which="both")
    ax.set_title(e)
plt.suptitle("PSD")

plt.show()
