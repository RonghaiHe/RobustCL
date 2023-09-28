[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![MIT License][license-shield]][license-url]
<!--[![LinkedIn][linkedin-shield]][linkedin-url]-->



<!-- PROJECT LOGO -->
<br />
<div align="center">
  <!--<a href="https://github.com/RonghaiHe/RobustCL">
    <img src="images/logo.png" alt="Logo" width="80" height="80">
  </a>-->

  <h3 align="center">Robust Cooperative localization simulation</h3>

  <p align="center">
    A simulation code for *Robust Cooperative Localization with Failed Communication and Biased Measurements*
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
    <li><a href="#roadmap">Roadmap</a></li>
    <!--<li><a href="#contributing">Contributing</a></li>-->
    <li><a href="#license">License</a></li>
    <li><a href="#contact">Contact</a></li>
    <li><a href="#acknowledgments">Acknowledgments</a></li>
  </ol>
</details>



<!-- ABOUT THE PROJECT -->
## Introduction

<!--[![Cover][cover]](https://github.com/RonghaiHe/RobustCL)-->

Cooperative Localization (CL) plays a crucial role in achieving precise localization without relying on localization sensors. However, the performance of CL can be significantly affected by **failed communication and biased measurements**. This code shows a robust decentralized CL method that addresses these challenges effectively.

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

<!--<p align="right">(<a href="#readme-top">back to top</a>)</p>-->

## Structure of Repositoy

Repository
- `algorithms`, a directory containing all algorithms used et al, **off the record temporarily**.
  - `BDA.py`
  - `CI.py`
  - `CU.py`
  - `DCL_GS.py`
  - `DMV.py`
  - `DR.py`
  - `func_detect_fault.py`
- `others`, a directory containing other calculation in the paper
  - `KL_calc.py`: calulate the numerical KL-divergence
  - `video_output.py`: multiple figures -> video
- `draw.py`: draw the final figures
- `main_fix.py`: mian function, set the fix running parameters here
- `parameters.py`: set the system parameters
- `requirements.txt`, a file containing all Python libraries needed.
- `shell_main.fix.sh`: a shell file to run the program simultaneously
- `utils.py`: execute the whole simulation

<!-- ROADMAP -->
## Roadmap

- [x] Complete Python project structure
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



<!-- LICENSE -->
## License

Distributed under the MIT License. See `LICENSE` for more information.

<!--<p align="right">(<a href="#readme-top">back to top</a>)</p>-->



<!-- CONTACT -->
## Contact

RonghaiHe - [E-mail](herh5@gmail.com)

Project Link: [RobustCL](https://github.com/RonghaiHe/RobustCL)

<!--<p align="right">(<a href="#readme-top">back to top</a>)</p>-->



<!-- ACKNOWLEDGMENTS -->
## Acknowledgments
* All authors of this paper;
* Inspired by [repository](https://github.com/RonghaiHe/multirobot_localization_tsangkai_utias);
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
