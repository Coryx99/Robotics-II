- RVIZ meshcat and drake viz

**05\_Diagrams and Result Visualization:**

-   **Creating Diagrams**

    -   Introduction to Drake\'s diagram system
    -   Building and visualizing diagrams

-   **Result Visualization**

    -   Tools and methods for visualizing simulation results
    -   Examples and best practices

**\[WIP\] Visualize data**
==========================

*Drake* could call python functions. So we could use matplotlib in
python to create data plots.

This piece of tool is used for translating the function call in C++ into
python and transmit some data format as well. We need to run the
*call\_python\_client\_cli* to enable this feature.

[]{#anchor}*cd drake*

*bazel build //common/proto:call\_python\_client\_cli
//common/proto:call\_python\_server\_test*

*\# Create default pipe file.*

*rm -f /tmp/python\_rpc && mkfifo /tmp/python\_rpc*

*\# Run the translation software.*

*./bazel-bin/common/proto/call\_python\_client\_cli*

So in our code, we could use this service and call *plot* in python to
draw figures.

[]{#anchor-1}So we could see the tool receive the python function call
request and data passed with this request. A figure pop up after the
plot command. This is how we plot data in *Drake*.
