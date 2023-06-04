# Open3D implementation of the paper Estimating Discrete Total Curvature with Per Triangle Normal Variation

A universal total curvature estimation method that works for both triangle meshes and point clouds. For details, see the 2023 SIGGRAPH paper by Crane Chen under the supervision of Misha Kazhdan.
![mesh_pcd_curvature](https://user-images.githubusercontent.com/33951209/229395487-efa580f7-9e28-498d-9265-af09d75f6d5c.png)

## Comparison with other popular libraries
![teaser_bright](https://user-images.githubusercontent.com/33951209/229387054-371fa8e9-1ef2-4552-81e3-af6927ee99dc.png)

## Dependencies

The code was developed on MacOS 12.6, adapting from the template code of open3d, [open3d-cmake-find-package](https://github.com/isl-org/open3d-cmake-find-package.git)

- [open3d](https://github.com/isl-org/Open3D.git)
- [STL](https://www.geeksforgeeks.org/the-c-standard-template-library-stl/)
- [eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page) for matrix data structures
- [openmp](http://polyscope.run/) for parallelization
- [cgal](https://www.cgal.org/) for delaunay triangulation


## Run

Just issue,

    git clone https://github.com/HeCraneChen/open3d-discrete-total-curvature.git --recursive
    cd open3d-discrete-total-curvature
    cd build
    ./Draw

**Compile**

Compile this project from scratch using the standard cmake routine (make sure that Open3D and CGAL are already installed on your machine):

    cd open3d-discrete-total-curvature
    rm -r build
    mkdir build
    cd build
    cmake ..
    make

## Citation

```bibtex
@article{chen2023estimating,
  title={Estimating Discrete Total Curvature with Per Triangle Normal Variation},
  author={Chen, Crane He},
  journal={arXiv preprint arXiv:2305.12653},
  year={2023}
}
```

