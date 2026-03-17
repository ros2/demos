# Copyright 2019 Canonical, Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from pathlib import Path

from ament_mypy.main import main

import pytest


@pytest.mark.mypy
@pytest.mark.linter
def test_mypy() -> None:
    def is_package_root(candidate: Path) -> bool:
        return (
            (candidate / 'package.xml').is_file()
            and (candidate / 'setup.py').is_file()
            and (candidate / 'lifecycle_py').is_dir()
            and (candidate / 'launch').is_dir()
            and (candidate / 'test').is_dir()
        )

    def find_package_root() -> Path:
        here = Path(__file__).resolve()
        cwd = Path.cwd().resolve()

        candidates = [
            here.parent.parent,
            cwd,
            cwd / 'lifecycle_py',
        ]

        for candidate in candidates:
            if is_package_root(candidate):
                return candidate

        # As a last resort, walk parents of __file__ and cwd.
        for candidate in [*here.parents, *cwd.parents]:
            if is_package_root(candidate):
                return candidate

        # Keep mypy scoped if no package root is detected.
        return here.parent.parent

    package_root = find_package_root()
    paths_to_check = [
        str(package_root / 'lifecycle_py'),
        str(package_root / 'launch'),
        str(package_root / 'test'),
        str(package_root / 'setup.py'),
    ]

    rc = main(argv=paths_to_check)
    assert rc == 0, 'Found code style errors / warnings'
