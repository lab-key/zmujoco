<h1>
  <a href="#"><img alt="zMuJoCo" src="zmujoco.png" width="100%"/></a>
</h1>

<p>
  </a>
  </a>
  <a href="https://github.com/lab-key/zmujoco/blob/main/LICENSE" alt="License">
    <img src="https://img.shields.io/github/license/lab-key/zmujoco">
  </a>
</p>


# About

This repository contains the Zig bindings for the MuJoCo C API.

Zig version 0.15.x

MuJoCo version 3.3.7

**MuJoCo** stands for **Mu**lti-**Jo**int dynamics with **Co**ntact. It is a
general purpose physics engine that aims to facilitate research and development
in robotics, biomechanics, graphics and animation, machine learning, and other
areas which demand fast and accurate simulation of articulated structures
interacting with their environment.

***For more higher level and a ( *little bit of a* ) Desktop app making framework checkout [phyzx] ( Still early stages )***

This repository contains a thin wrapper for MuJoCo allowing for easier integration into other projects and direct programmatic usage of the C API.

It uses my fork of MuJoCo if you want to use your own fork of MuJoCo checkout the upstream project and instructions there, it's pretty simple!

MuJoCo has a C API and is intended for researchers and developers. The runtime
simulation module is tuned to maximize performance and operates on low-level
data structures that are preallocated by the built-in XML compiler. 

MuJoCo further exposes a large number of utility functions for computing
physics-related quantities.



## Getting Started

You should be able to just:

```
git clone https://github.com/lab-key/zmujoco.git

zig build
```

## TODO

Examples & tests a nice clean way to integrate or point to third party modelfiles. 

## Documentation

All the documenatation is in the docs/ directory.

I highly recommend you look at the tests directory as well as it contains a lot of examples as does the phyzx[link] project.

MuJoCo's documentation can be found at [mujoco.readthedocs.io]. Upcoming
features due for the next release can be found in the [changelog] in the
"latest" branch.

## Citation

If you use MuJoCo for published research, please cite:

```
@inproceedings{todorov2012mujoco,
  title={MuJoCo: A physics engine for model-based control},
  author={Todorov, Emanuel and Erez, Tom and Tassa, Yuval},
  booktitle={2012 IEEE/RSJ International Conference on Intelligent Robots and Systems},
  pages={5026--5033},
  year={2012},
  organization={IEEE},
  doi={10.1109/IROS.2012.6386109}
}
```

## License and Disclaimer

Copyright 2021 DeepMind Technologies Limited.

Box collision code ([`engine_collision_box.c`](https://github.com/google-deepmind/mujoco/blob/main/src/engine/engine_collision_box.c))
is Copyright 2016 Svetoslav Kolev.

ReStructuredText documents, images, and videos in the `doc` directory are made
available under the terms of the Creative Commons Attribution 4.0 (CC BY 4.0)
license. You may obtain a copy of the License at
https://creativecommons.org/licenses/by/4.0/legalcode.

Source code is licensed under the Apache License, Version 2.0. You may obtain a
copy of the License at https://www.apache.org/licenses/LICENSE-2.0.

This is not an officially supported Google product.

[phyzx]: https://github.com/lab-key/phyzx
[build from source]: https://mujoco.readthedocs.io/en/latest/programming#building-mujoco-from-source
[Getting Started]: https://mujoco.readthedocs.io/en/latest/programming#getting-started
[Unity]: https://unity.com/
[releases page]: https://github.com/google-deepmind/mujoco/releases
[mujoco.readthedocs.io]: https://mujoco.readthedocs.io
[changelog]: https://mujoco.readthedocs.io/en/latest/changelog.html
[Python bindings]: https://mujoco.readthedocs.io/en/stable/python.html#python-bindings
[PyPI]: https://pypi.org/project/mujoco/
