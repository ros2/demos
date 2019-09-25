# Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

import ament_copyright.main
import ament_flake8.main
import ament_pep257.main

import pytest


@pytest.mark.copyright
@pytest.mark.linter
def test_copyright():
    # Test is called from package root
    rc = ament_copyright.main.main(argv=['.'])
    assert rc == 0, 'Found copyright errors'


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    # Test is called from package root
    rc = ament_flake8.main.main(argv=['.'])
    assert rc == 0, 'Found flake8 errors'


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    # Test is called from package root
    rc = ament_pep257.main.main(argv=['.'])
    assert rc == 0, 'Found PEP257 code style error / warnings'
