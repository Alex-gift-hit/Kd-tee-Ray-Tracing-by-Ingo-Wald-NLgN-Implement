# 1 Perface
The [KD tree O(NlgN) algorithm of Ingo Wald](https://www.sci.utah.edu/~wald/Publications/2006/NlogN/download/kdtree.pdf) is Implemented (May be some bugs or flaws, I don't know, It work properly.). But the performace is much worse than that of BVH.

This project start at April 2023, and finished at June 2023. But now is 2024.

Although this work is not that difficult, but there is not a lot of relevant information, and it takes a lot of effort to do it.

However, the main purpose of this code is homework, so I hope you don't just copy it, but respect the code. 

I also hope to give some help to the walkers who are looking for KD trees to implement ray tracing.

Thank  to everyone who shared the knowledge.
# 2 Ref

**The structure of the code, as well as the main content comes from Games103 2019 Linqi Yan.**

But about the part of Kd-tree, I refer to a lot of sources.

* [Apr 11, 2018 Roman Wiche : How to create awesome accelerators: The Surface Area Heuristic](https://medium.com/@bromanz/how-to-create-awesome-accelerators-the-surface-area-heuristic-e14b5dec6160)

* [2015 CMU 15-462/622: Accelerating Geometric Queries](http://15462.courses.cs.cmu.edu/fall2015/lecture/acceleration/slide_024)

* UC Berkeley CS184 (But the content isn't reachable for now.)

* [(23 November 2004) Jacco Bikker  Raytracing Topics & Techniques - Part 7: Kd-Trees and More Speed](https://www.flipcode.com/archives/Raytracing_Topics_Techniques-Part_7_Kd-Trees_and_More_Speed.shtml)

* [6.837 MIT autum 2020](https://www.youtube.com/watch?v=TrqK-atFfWY) 
** [Lecture Note](https://ocw.mit.edu/courses/6-837-computer-graphics-fall-2012/51937f370603b18f259f00e814f96a0c_MIT6_837F12_Lec14.pdf)

* [September 2006 Ingo Wald†, Vlastimil Havran : On building fast kd-Trees for Ray Tracing, and on doing that in O(N log N)](https://www.sci.utah.edu/~wald/Publications/2006/NlogN/download/kdtree.pdf)

# 3 Impl
There are two types of kd-tree based on stop spliting rule.

Modify it at here.`SAHBuildNode* BVHAccel::recursiveSAHBuild(std::vector<Object*> objects, EventsXYZ Events, Bounds3 bounds, int kdDepth)`
1. Auto stop.(Even worse)
2. Max split times. (Better choice)

All kd-tree based ray-tracing Agl are two steps.
1. Build the tree.
2. Intersect the ray and box.

Three core files.
* Bounds3.hpp
* BVH.cpp
* BVH.hpp