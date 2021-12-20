See `lifecycle/README.rst <../lifecycle/README.rst>`_.
You can follow the same instructions and replace:

.. code-block:: python

    ros2 run lifecycle lifecycle_talker

with:

.. code-block:: python

    ros2 run lifecycle_py lifecycle_talker


You still need to use the ``lifecycle_listener`` and ``lifecycle_service_client`` executables in the ``lifecycle`` package, as those demos were not ported to python.
