^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package diagnostic_recorder
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.2 (2022-01-14)
------------------
* Escape double quotes in csv file.
* Contributors: Patrick Chin

0.1.1 (2021-05-17)
------------------
* Insert Unicode BOM in the output csv file.
* Contributors: Patrick Chin

0.1.0 (2019-02-17)
------------------
* Upgrade the format of package.xml to v2.
* Fix linter errors.
* Fix missing data header in a newly rotated file. (since 45eafca2bc)
* Make filesystem storage rewrite the data header when the header has changed.
* Add filesystem storage backend.
* Initial commit.
* Contributors: Patrick Chin
