ClusterPointIndicesToPointIndices
=================================


What is this?
-------------

Convert cluster indices to point indices with specified index value.


Subscribing Topic
-----------------

* ``~input`` (``jsk_recognition_msgs/ClusterPointIndices``)

  Cluster indices.

Publishing Topic
----------------

* ``~output`` (``pcl_msgs/PointIndices``)

  Output point indices.


Parameters
----------

* ``~index`` (Int, default: `-1`)

  Index value where point indices is extracted from cluster indices.
  Please note that negative index is skipped, and empty indices is published.
