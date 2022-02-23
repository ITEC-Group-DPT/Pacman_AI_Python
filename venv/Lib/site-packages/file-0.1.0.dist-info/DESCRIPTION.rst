========
Overview
========



CFFI bindings for libmagic.

* Free software: BSD 2-Clause License

Installation
============

::

    pip install file

Documentation
=============

https://python-file.readthedocs.io/

Development
===========

To run the all tests run::

    tox

Note, to combine the coverage data from all the tox environments run:

.. list-table::
    :widths: 10 90
    :stub-columns: 1

    - - Windows
      - ::

            set PYTEST_ADDOPTS=--cov-append
            tox

    - - Other
      - ::

            PYTEST_ADDOPTS=--cov-append tox


Changelog
=========

0.1.0 (2017-11-02)
------------------

* First release on PyPI.


