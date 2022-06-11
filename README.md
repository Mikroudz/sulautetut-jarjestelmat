
<!-- PROJECT LOGO -->
<br />
<div align="center">

<h3 align="center">Balancing robot</h3>

  <p align="center">
    Two wheeled balancing robot using 6DoF accelerometer and gyroscope.
  </p>
</div>



<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
      <ul>
        <li><a href="#built-with">Built With</a></li>
      </ul>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="#usage">Usage</a></li>
    <li><a href="#roadmap">Roadmap</a></li>
    <li><a href="#contributing">Contributing</a></li>
    <li><a href="#license">License</a></li>
    <li><a href="#contact">Contact</a></li>
    <li><a href="#acknowledgments">Acknowledgments</a></li>
  </ol>
</details>



<!-- ABOUT THE PROJECT -->
## About The Project

[![Product Name Screen Shot][product-screenshot]](https://example.com)


<p align="right">(<a href="#top">back to top</a>)</p>


### Built With

* [Visual Studio Code](https://code.visualstudio.com/)
* [Platformio IO](https://platformio.org/)
* [STM32PIO](https://github.com/ussserrr/stm32pio)
* [CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html)

<p align="right">(<a href="#top">back to top</a>)</p>



<!-- GETTING STARTED -->
## Getting Started

Easiest way to develop is with Visual Studio Code and Platformio. Template file generated with CubeMX and converted to VSCode format with STM32PIO. All tools work in both linux and windows operating systems. To program and debug the PCB, stlink interface is used and it requires a Nucleo development board or a Stlink programmer.


### Installation

1. Install Visual Studio Code for your distribution

2. Install Platformio IO inside Visual Studio Code.

3. Clone the project to your local PC
   ```sh
   git clone https://github.com/Mikroudz/sulautetut-jarjestelmat.git
   ```
4. Open project folder "STM32F410" in Visual Studio Code. Platformio should detect the project and install required compilation tools. Wait till it finishes.

## Programming the device
1. Connect a JTAG programming interface to the device and a Nucleo programmir header or JTAG programmer.

2. Power up the PCB with 11 - 16 volt power supply.

3. Insert USB to a pc with Visual Studio and platformio installed. The STM32 programmer should be detected.

4. Compile and program the device by pressing "Program" button in bottom of VSCode screen.

5. If successful the robots led should blink slowly.

<p align="right">(<a href="#top">back to top</a>)</p>

### Making configuration changes

To change I/O port configurations or device clocks, the project has been configured to use CubeMX tool. 
WARNING: if configurations are changed directly in the code, they will be overwritten by CubeMX.

1. Install CubeMX and open STM32F410.ioc file in the program.

2. Make changes and save the file.

3. Run [STM32PIO](https://github.com/ussserrr/stm32pio) on command line to update the project files:
   ```sh
   stm32pio generate -d path/to/ioc/file/folder
   ```

<!-- USAGE EXAMPLES -->
## Usage

After powering the robot and without LoRa connection it should be ready to balance. If the leds are blinking then hold the robot in upright position until it start to balance. If robot falls over too much forward or backwards, it stops the motors. 

<p align="right">(<a href="#top">back to top</a>)</p>



<!-- LICENSE -->
## License

Distributed under the MIT License. See `LICENSE.txt` for more information.

<p align="right">(<a href="#top">back to top</a>)</p>


<!-- ACKNOWLEDGMENTS -->
## Acknowledgments

* [@Mikroudz](https://github.com/Mikroudz)
* [@jruntti20](https://github.com/jruntti20)
* [@valtteriah](https://github.com/valtteriah)

<p align="right">(<a href="#top">back to top</a>)</p>
