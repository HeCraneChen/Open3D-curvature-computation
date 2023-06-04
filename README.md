# Open3D -- Estimating Discrete Total Curvature with Per Triangle Normal Variation

A universal total curvature estimation method that works for both triangle meshes and point clouds. For details, see the 2023 SIGGRAPH paper by Crane Chen under the supervision of Misha Kazhdan.

This codebase was developed on MacOS 12.6, adapting from the template code of open3d, [open3d-cmake-find-package](https://github.com/isl-org/open3d-cmake-find-package.git), original research code can be found [here](https://github.com/HeCraneChen/total-curvature-estimation.git).
<img width="809" alt="o3d_curvature_teaser" src="https://github.com/HeCraneChen/open3d-discrete-total-curvature/assets/33951209/3297f9f8-9811-42de-8650-41397a10d3cc">

## Comparison with other popular libraries
![teaser_bright](https://user-images.githubusercontent.com/33951209/229387054-371fa8e9-1ef2-4552-81e3-af6927ee99dc.png)

## Dependencies

- [open3d](https://github.com/isl-org/Open3D.git)
- [STL](https://www.geeksforgeeks.org/the-c-standard-template-library-stl/)
- [eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page) for matrix data structures
- [openmp](http://polyscope.run/) for parallelization
- [cgal](https://www.cgal.org/) for delaunay triangulation


## Run

On MacOS, just issue (make sure that Open3D and CGAL are already installed on your machine, use Homebrew to install CGAL on MacOS):

    git clone https://github.com/HeCraneChen/open3d-discrete-total-curvature.git --recursive
    cd open3d-discrete-total-curvature
    cd build
    ./Draw
    ![o3d_curvature_output](https://github.com/HeCraneChen/open3d-discrete-total-curvature/assets/33951209/93043526-1115-48da-bd5e-2e7d3771d938)


**Compile**

Compile this project from scratch using the standard cmake routine (make sure that Open3D and CGAL are already installed on your machine, use Homebrew to install CGAL on MacOS):

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

