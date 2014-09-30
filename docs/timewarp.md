For example, this hierarchy of DSDE models:
             C10
          /   |    \
         /    |     \
        /     |      \
     C11     C12      C13
     / \     / \    /  |  \
    C   C   C   C   C  C   A

Becomes, for example, with 3 logical processors:
     LP1        LP2       LP3
     / \         |         |
    A  C10      C12       C13

LP1 manages the atomic model A and the coupled model C10. A and C10
are not connected and the current time of these model can be
asynchornous. So, for each children of a logical processor, user
define a float to define the computation priority. If `A.processor =
0.1` and `C10.processor = 0.9` then kernel ensures that C10 will be
called ten times more that A.
