ColorHistogramFilter
==============

![color_histogram](https://cloud.githubusercontent.com/assets/1901008/26710312/f1343f54-4793-11e7-8848-9a2f31e4b5d5.png)

Filter point indices using color histogram by comparing with reference histogram

Methods for histogram comparison is configurable from multiple methods. (See parameter `~compare_policy`)
After computing distance between input histograms and reference, filter by thresholding (See parameter `~distance_threshold`)

## Subscribing Topics

* `~input` (`jsk_recognition_msgs/ColorHistogramArray`)

    Input color histogram array  
    The order of each histograms must be the same as the order of input cluster point indices.

* `~input/indices` (`jsk_recognition_msgs/ClusterPointIndices`)

    Input point indices

## Publishing Topics

* `~output` (`jsk_recognition_msgs/ColorHistogramArray`)

    Filtered color histogram array

* `~output/indices` (`jsk_recognition_msgs/ClusterPointIndices`)

    Filtered cluster point indices

## Parameters

* `~queue_size` (`Int`, default: `100`)

    Queue size for message synchronization

* `~bin_size` (`Int`, default: `100`)

    Bin size for histogram

* `~compare_policy` (`Enum[Int]`, default: `CORRELATION`)

    Policy for histogram values to compare

    - 0: `CORRELATION`
        - Use correlation
    - 1: `BHATTACHARYYA`
        - Use bhattacharyya distance
    - 2: `INTERSECTION`
        - Use vector intersection
    - 3: `CHISQUARE`
        - Use chi-square between two vectors
    - 4: `KL_DIVERGENCE`
        - Use Kullback-Leibler divergence for comparing two vectors
