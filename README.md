# Reference Governor Safe Control with SDDM

This repository contains code for ICRA 2020 `Fast and Safe Path-Following Control using a State-Dependent Directional Metric`.  Reference governor framework equipped with novel metric enables fast and safe navigation in unknown complex environments. This code is primarily developed by Zhichao Li, a PhD student in the [Exsitential Robototics Laboratory](http://erl.ucsd.edu/).

## Dependency

The code is test on `18.04 LTS` on a desktop computer (Intel i7-8700K CPU  + 32 GB RAM) `Anaconda (Python 3.9)`. 
I will list the most important packages, you can check all packages in `rg_sddm39.yml`.

* third-party packages
  + [vtkplotter 2019.4.4](https://vtkplotter.embl.es/)
  + [mosek solver 9.0](https://www.mosek.com/)
  + [cvxpy 1.0.24](https://www.cvxpy.org/)
  + [trimesh 3.1.5](https://github.com/mikedh/trimesh)

* Other dependencies
  + A* planner compile and copy `*.so` to `/src/mylib` see `README` in `astar_py` folder
  + Eigen3  `sudo apt install -y libeigen3-dev`
  + Boost `sudo apt install-y libboost-dev`

To replicate the testing environment, we recommend using `Anaconda` to create
a virtual environment as follows. For both approaches, you need to obtain a
[MOSEK solver license](https://www.mosek.com/products/academic-licenses/) (Personal Academic License) and install it properly according to the instruction you received with the license. 


### Mannual setup

  ```sh
  conda create --name rg_sddm39 python=3.9
  conda activate rg_sddm39
  pip install -r requirements.txt
  conda install -c mosek mosek
  ```

## Usage

To replicate the main result, please use the following instruction. I avoid using arg parser and make the code very easy to achieve different functionalities by just toggling comments of certain lines. For simplicity, the usage is omitted, and more detail can be found in the main functions.
Run all codes within `src` folder. All log files are save in `log` folder and all figures are saved in `sim_figs` folder.

* Evaluate sparse known circular environment
  + Baseline Euclidean norm **c1=1, c2=1** (**change line 45**) , [about 15 secs].

    ```py
    python main_sparse_known.py
    ```