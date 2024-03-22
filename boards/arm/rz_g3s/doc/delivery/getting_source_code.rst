Getting Source Code
===================

Follow Zephyr Getting Started Guide: :ref:`getting_started`,
but change the one step from “Get the Zephyr source code”:

Instead of using this command west init *˜/zephyrproject* use the next one from your working directory:

.. code-block:: bash

    west init -m git@gitbud.epam.com:rec-rzzp/zephyr.git --mr rzg3s_dev zephyr_rzg3s
    cd zephyr_rzg3s/zephyr

Cloning GIT repo for the first time requires SSH keys to be registration. Please follow this guide for the detailed
instructions:

https://gitbud.epam.com/help/user/ssh.md
