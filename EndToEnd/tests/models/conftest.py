"""モデル検証だけに適用する、再現可能なCPU実行設定。"""

import pytest


@pytest.fixture(autouse=True)
def cpu_threads():
    torch = pytest.importorskip("torch")
    previous = torch.get_num_threads()
    torch.set_num_threads(1)
    torch.manual_seed(10)
    yield
    torch.set_num_threads(previous)
