^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package roswww
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.8 (2015-12-14)
------------------
* [doc] Add rostopic-chat sample `#31 <https://github.com/tork-a/roswww/issues/31>`_
* [sys] Utilize test_depend that is defined in REP-140 `#30 <https://github.com/tork-a/roswww/issues/30>`_ from 130s/impr/utilize_testdepend
* Contributors: Kenta Yonekura, Isaac I.Y. Saito

0.1.7 (2015-12-07)
------------------
* [fix] Developers don't need to create a sample www directory now that it's installed (`#28 <https://github.com/tork-a/roswww/issues/28>`_)
* [fix][test] Use install space for testing (to capture the issue raised in `#28 <https://github.com/tork-a/roswww/issues/28>`_)
* Contributors: Kenta Yonekura, Isaac I.Y. Saito

0.1.6 (2015-12-01)
------------------
* [sys] Add travis config and small unit tests (`#27 <https://github.com/tork-a/roswww/issues/27>`_)
* [doc] Utilize rosdoc_lite for document generation on ros build farm (`#25 <https://github.com/tork-a/roswww/issues/25>`_)
* Contributors: Isaac I.Y. Saito

0.1.5 (2015-02-28)
------------------
* Install missing launch directory
* Contributors: Jihoon Lee

0.1.4 (2015-01-28)
------------------
* example launchfile. update dependency
* configurable package web root
* add parmeter in readme
* classfy roswww webserver. remove ros runtime dependency
* Contributors: Jihoon Lee, Isaac I.Y. Saito

0.1.3 (2015-01-05)
------------------
* Separate webserver.py into module and script files (quick fix to `#10 <https://github.com/tork-a/roswww/issues/10>`_).
* Contributors: Isaac I.Y. Saito, Jihoon Lee

0.1.2 (2014-12-13)
------------------
* Re-release into ROS (addresses `#1 <https://github.com/tork-a/roswww/issues/3>`_)
* Remove tornado (this dependency is supposed to be taken from rosbridge). Move webserver.py to src folder to follow more common python style (fix `#1 <https://github.com/tork-a/roswww/issues/1>`_)
* Add dependency on rosbridge, webserver.py installation.
* Remove roswww_pkg metapkg and roswww_pack that doesn't seem to be used. Remove redundant hierarchy.
* Code cleaning, conform to PEP8, refactor method names. Add docroot.
* Contributors: Isaac IY Saito

0.1.1 (2013-11-15)
------------------
* roswww) correction in response to the error on buildfarm
* roswww) change maintainer. Remove unncessary file
* Contributors: Isao Isaac Saito

0.1.0 (2013-11-14)
-----------
* Catkinize roswww and roswww_pack packages. Change repository from that of jihoonl to tork-a.
* Contributors: Isao Isaac Saito, Jihoon Lee, furushchev
