Getting Source Code
===================

Follow Zephyr Getting Started Guide: :ref:`getting_started`,
but change the one step from “Get the Zephyr source code”:

**NOTE** Please install Zephyr SDK version 0.16.5-1 or later.

Instead of using this command west init *˜/zephyrproject* use the next one from your working directory:

.. code-block:: bash

    west init -m git@github.com:xen-troops/zephyr.git --mr rzg3-release-v1.0.2 zephyr_rzg3s
    cd zephyr_rzg3s/zephyr

Cloning GIT repo for the first time requires SSH keys to be registration. Please follow this guide for the detailed
instructions:

`Generating SSH Keys <https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent>`_
