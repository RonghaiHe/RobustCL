<div align="center">
  
  [![Contributors][contributors-shield]][contributors-url]
  [![Forks][forks-shield]][forks-url]
  [![Stargazers][stars-shield]][stars-url]
  [![Issues][issues-shield]][issues-url]
  [![MIT License][license-shield]][license-url]
<!--[![LinkedIn][linkedin-shield]][linkedin-url]-->
</div>


<!-- PROJECT LOGO -->

<div align="center">
  <!--<a href="https://github.com/RonghaiHe/RobustCL">
    <img src="images/logo.png" alt="Logo" width="80" height="80">
  </a>-->

  <h3 align="center">Robust Cooperative localization simulation</h3>

  <p align="center">
    A simulation code for the paper "<em>Robust Cooperative Localization with Failed Communication and Biased Measurements</em>",
    <br />
    published in <a href="https://ieeexplore.ieee.org/document/10423111">IEEE Robotics and Automation Letters</a> with a supplemental video that is also in <a href="https://www.bilibili.com/video/BV14C411z7iM/">bilibili</a>
    <!--<br />
    <a href="https://github.com/RonghaiHe/RobustCL"><strong>Explore the docs »</strong></a>
    <br />-->
    <br />
    <a href="https://github.com/RonghaiHe/RobustCL">View Demo</a>
    ·
    <a href="https://github.com/RonghaiHe/RobustCL/issues">Report Bug OR Request Feature</a>
  </p>
</div>



<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#introduction">Introduction</a>
      <ul>
        <!--<li><a href="#built-with">Built With</a></li>
      </ul>-->
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <!--<li><a href="#prerequisites">Prerequisites</a></li>-->
        <!--<li><a href="#installation">Installation</a></li>
      </ul>-->
    </li>
    <li><a href="#usage">Usage</a></li>
    <li><a href="#structure">Structure</a></li>
    <li><a href="#roadmap">Roadmap</a></li>
    <!--<li><a href="#contributing">Contributing</a></li>-->
    <li><a href="#errata">Errata</a></li>
    <li><a href="#license">License</a></li>
    <li><a href="#contact">Contact</a></li>
    <li><a href="#acknowledgments">Acknowledgments</a></li>
  </ol>
</details>



<!-- ABOUT THE PROJECT -->
## Introduction

<!--[![Cover][cover]](https://github.com/RonghaiHe/RobustCL)-->

Cooperative Localization (CL) plays a crucial role in achieving precise localization without relying on localization sensors. However, the performance of CL can be significantly affected by **failed communication and biased measurements**. This code shows a robust decentralized CL method that addresses these challenges effectively.

Please cite this work when referencing:
```
@ARTICLE{RobustCL_RAL,
  author={He, Ronghai and Shan, Yunxiao and Huang, Kai},
  journal={IEEE Robotics and Automation Letters}, 
  title={Robust Cooperative Localization With Failed Communication and Biased Measurements}, 
  year={2024},
  volume={9},
  number={3},
  pages={2997-3004}
}
```

About this paper: there are a few mistakes shown in the <a href="#errata">Errata</a> part.

<!--<p align="right">(<a href="#readme-top">back to top</a>)</p>-->


<!-- GETTING STARTED -->
## Getting Started

1. Since this is a Python program, you will need **Python3**. Please Make sure you have it installed.
2. Simply clone the repo
   ```sh
   git clone https://github.com/RonghaiHe/RobustCL.git
   ```
3. Install the Python libraries that this program depends on through `requirements.txt`:
   ```sh
   pip install -r requirements.txt
   ```

<!--<p align="right">(<a href="#readme-top">back to top</a>)</p>-->

<!-- USAGE EXAMPLES -->
## Usage

- If runinng the program with specific failed communication probability(e.g. 0.5):
  ```sh
  python3 main_fix.py --comm_fail_prob 0.5
  ```
- If runinng the program with multiple failed communication probability simultaneously:
  ```sh
  bash shell_main_fix.sh
  ```
- If runinng the **mission**(e.g. 1) with multiple failed communication probability simultaneously:
  ```sh
  bash shell_main.sh 1
  ```

  Details of missions are:

  `0` Draw the sample trajectory, for Response;

  `1` ARMSE over time, for Fig. 1 & Fig. 2 in the paper;

  `2` weight of M-estimation over time, for Fig. 3 in the paper;

  `3` ARMSE over tau, for Fig. 4 in the paper;

  `4` animation about ARMSE over time, for videos about the paper;

  `5` animation about ARMSE over tau, for videos about the paper

<!--<p align="right">(<a href="#readme-top">back to top</a>)</p>-->

## Structure

Repository contains: 
- `algorithms`, a directory containing all algorithms used et al, ~~off the record temporarily~~**up to date**.
  - `BDA.py`: "BDA" algorithm in the paper;
  - `CI.py`: "CI" algorithm in the paper;
  - `CU.py`: "BDA-CU", "CI+CU" algorithms in the paper;
  - `DCL_GS.py`: Our proposed method in the paper;
  - `DMV.py`: "DMV" algorithm in the paper;
  - `DR.py`: "DR" method in the paper;
  - `func_detect_fault.py`: multiple functions about detecting the fault measurement;
- `others`, a directory containing other calculation in the paper
  - `KL_calc.py`: calulate the numerical KL-divergence;
  - `video_output.py`: multiple figures -> video;
- `draw.py`: draw the final figures;
- `main_fix.py`: main function. Set the fix running parameters here;
- `parameters.py`: sets the system parameters;
- `requirements.txt`, a file containing all Python libraries needed;
- `shell_main.fix.sh`: a shell file to run the program simultaneously;
- `utils.py`: execute the whole simulation

<!-- ROADMAP -->
## Roadmap
- [x] Whole codes uploaded
- [x] Formatted with pep8
- [ ] Complete Python project structure

See `Changelog.md` for the change at each version
<!--- [x] Add Changelog-->

<!--<p align="right">(<a href="#readme-top">back to top</a>)</p>-->



<!-- CONTRIBUTING -->
<!--## Contributing

Contributions are what make the open source community such an amazing place to learn, inspire, and create. Any contributions you make are **greatly appreciated**.

If you have a suggestion that would make this better, please fork the repo and create a pull request. You can also simply open an issue with the tag "enhancement".
Don't forget to give the project a star! Thanks again!

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

<!--<p align="right">(<a href="#readme-top">back to top</a>)</p>-->

## Errata
The conclusion of the paper isn't wrong, whereas there are a few mistakes in this paper. Sorry about that and declare here:

1. In formulas (17) and (20), the dimension of $\boldsymbol{I}$ should be $3N\times 3N$ rather than $N\times N$ (By Chang Liu)
2. Formula (26) is wrong, $\boldsymbol{S}_ {t+1} \to \boldsymbol{M}_{t+1}$ (By Chang Liu)
3. (Confusion Part)In the formula (23), the variable $\hat{\boldsymbol{X}}_ {t+1 \mid t}^i$ is shown for explanation. During the iteration, it is replaced by the result after $n$-th iterations. Specifically, we use the latest iteration result, `X_all`, to calculate the residual item in this <a href="https://github.com/RonghaiHe/RobustCL/blob/50a3c3ff5e449491687bc112af22b4804b0d721a/algorithms/DCL_GS.py#L872">code</a> (By Chang Liu) 
4. The serial numbers of references used for comparison in Fig. 1, Fig. 2, Fig. 4 and TABLE II haven't been updated. The **right** serial numbers can be seen in the last paragraph of section V-A: `DMV [13], BDA [12], BDA-CU [4] and CI+CU [17]` (By a professor).

Still updating ...

<!-- LICENSE -->
## License

Distributed under the MIT License. See `LICENSE` for more information.

<!--<p align="right">(<a href="#readme-top">back to top</a>)</p>-->



<!-- CONTACT -->
## Contact

RonghaiHe - [E-mail](mailto:herh5@gmail.com)

Project Link: [RobustCL](https://github.com/RonghaiHe/RobustCL)

<!--<p align="right">(<a href="#readme-top">back to top</a>)</p>-->



<!-- ACKNOWLEDGMENTS -->
## Acknowledgments
* All authors of this paper;
* Readers pointing out errors or improvements about the paper: professors, Chang liu;
* Inspired by this [repository](https://github.com/RonghaiHe/multirobot_localization_tsangkai_utias);
* A recommended [template](https://github.com/othneildrew/Best-README-Template/tree/master). This README.md is modified from that.

<!--<p align="right">(<a href="#readme-top">back to top</a>)</p>-->



<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/RonghaiHe/RobustCL.svg?style=for-the-badge
[contributors-url]: https://github.com/RonghaiHe/RobustCL/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/RonghaiHe/RobustCL.svg?style=for-the-badge
[forks-url]: https://github.com/RonghaiHe/RobustCL/network/members
[stars-shield]: https://img.shields.io/github/stars/RonghaiHe/RobustCL.svg?style=for-the-badge
[stars-url]: https://github.com/RonghaiHe/RobustCL/stargazers
[issues-shield]: https://img.shields.io/github/issues/RonghaiHe/RobustCL.svg?style=for-the-badge
[issues-url]: https://github.com/RonghaiHe/RobustCL/issues
[license-shield]: https://img.shields.io/github/license/RonghaiHe/RobustCL.svg?style=for-the-badge
[license-url]: https://github.com/RonghaiHe/RobustCL/blob/master/LICENSE.txt
[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=for-the-badge&logo=linkedin&colorB=555
[linkedin-url]: https://linkedin.com/in/othneildrew
[cover]: docs/cover.png
