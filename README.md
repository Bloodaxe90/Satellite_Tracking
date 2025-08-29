<h1 align="center">Satelite Tracking</h1>

<h2>Description:</h2>
<p>
  
</p>

<h2>Setup:</h2>
  <b>Camera</b>:
  <ol>
    <li>Visit www.zwoastro.com/software</li>
    <li>
      Download and install ASIStudio and the appropriate camera driver, the ASICap software is useful for fiddling around with settings
    </li>
    <li>On the same page, got to the "Others" tab</li>
    <li>Locate and download the ASI Camera SDK from develop tools</li>
    <li>Interface with the SDK using the ZWOASI Python library available here https://github.com/python-zwoasi</li>
  </ol>
  <b>Optotune FSM:</b>
  <ol>
    <li>Check computer sees the input device. i.e. ls /dev/cu.* (mac terminal)</li>
    <li>Connect Fast Steering Mirror to controller</li>
    <li>Download a hyperterminal to interact with controller i.e. CoolTerm</li>
    <li>Check the terminals setting match the following:
      <ul>
        <li>Baud Rate: 25600</li>
        <li>Data Bits: 8</li>
        <li>Stop Bits: 1</li>
        <li>Parity: None </li>
      </ul>  
    </li>
    <li>Make sure “Local echo” is enabled and “Enter Key Emulation” is set to CR + LF</li>
    <li>Send ‘start’ to check the fsm is connected, OK should be returned</li>
    <li>Refer to optotune_fsm_manual found in the hardware_manuals folder in this shared drive for more information</li>
  </ol>
  <b>Code:</b>
  <ol>
    <li>Activate a virtual environment.</li>
    <li>Run <code>pip install -r requirements.txt</code> to install the dependencies.</li>
    <li>Edit the input parameters in the <code>config.yaml</code> file</li>
    <li>Run <code>main.py</code> to either tune the kalman filter or start tracking.</li>
  </ol>  
  <b>Model:</b>
  
  <div style="display: flex; justify-content: space-around; align-items: center;">
    <img width="42%" alt="image" src="https://github.com/user-attachments/assets/0e07f7a0-8345-44b4-8b4e-0538e31a8ccd" />
    <img width="48%" alt="image" src="https://github.com/user-attachments/assets/11cfd4fe-7222-4ce4-9943-d40618789b47" />
  </div>



<h2>Results:</h2>

<p>
  All raw result csv files can be found in the 'results' directory
</p>
<p>
  <b>Image noise reduction:</b>

  The below images show the following:
  <ul>
    <li>Left: Raw frame</li>
    <li>Middle: Raw frame with dark frame subtraction applied</li>
    <li>Right: Raw frame with dark frame subtraction and morphological opening applied</li>
  </ul>

  <img width="1101" height="320" alt="image" src="https://github.com/user-attachments/assets/c764d708-3679-44e9-9f8a-3926cff53792" />
</p>

<p>
  <b>Identifying laser position:</b>

  The below images show the following:
  <ol>
    <li>Image 1: Clean frame (image 1)</li>
    <li>Image 2: Clean frame with contours outlined</li>
    <li>Image 3: Clean frame with largest contour outlined and its center pin pointed </li>
  </ol>
  
  <img width="1428" height="352" alt="image" src="https://github.com/user-attachments/assets/a3916388-4a81-497e-8e90-a6e29302c978" />
</p>

<p>
  <b>Finding the minimum laser power at which the laser could be identified:</b>
  <ul>
    <li>
      For each laser power I measured the area, mean intensity and standerd deviation intensity. I also decided to see how well the laser could reproduce some input noise (a square wave with a frequency of 1 Hz), to do this I calculated the dominant frequency of the FFT of the pixels position over time. The frequency error plotted against laser power can also be seen below.
      <img width="1389" height="990" alt="clipboard706" src="https://github.com/user-attachments/assets/980676aa-693e-42d9-99af-b96c717495d4" />
    </li>
    <li>
      Since the mean intensity and standard deviation of the detected laser depend on the beam profiles measured area (as higher laser intensity causes the profile to “bloom” and include more of the faint halo), these raw metrics become misleading: the mean is artificially lowered, and the standard deviation inflated.
      <p>
        To remove this area dependency, two normalized quantities were introduced and plotted against the laser current:
        <ul>
          <li>
            Total Flux (Mean Intensity × Area): Provides a true measure of the total captured light energy, independent of blob size.
          </li>
          <li>
            Coefficient of Variation (CV = Standard Deviation / Mean): Quantifies the relative variability of pixel intensities, offering a measure of beam uniformity independent of brightness and area.
          </li>
        </ul>
      <img width="1390" height="490" alt="clipboard1441" src="https://github.com/user-attachments/assets/8a06ef88-2cf0-46d5-8f04-d57945c9c7db" />
      </p>
    </li>
  </ul>
</p>

<p>
  <b>Tracking the laser:</b> 
  <br>
  Here the fsm was used to move the laser beam at a constant velocity across the frame to simulate the satelites movement, occastionally the beam was turned off to simulate some obstruction (i.e. a cloud passing overhead) to allow the Kalman Filter to kick in and estimate the following consecutive locations of the laser (satelite)
  </br>

  Below are the results for the Minimum Power (0.0338 uW) and then a Baseline Visible Power (5.55 uW) initially showing the X position plotted against the Y positions and then the X and Y position both individually plotted against time all for both the estimated (Kalman Filter ON) and measured (Kalman Filter OFF) values.

  <ul>
    <li> Minimum Power (0.0338 uW):
      <div style="display: flex; justify-content: space-around; align-items: center;">
        <img width="45%" alt="clipboard2075" src="https://github.com/user-attachments/assets/21ec492d-bdf9-4c0e-af64-f46842a4fbb2" />
        <img width="45%" alt="clipboard1165" src="https://github.com/user-attachments/assets/6f4cff0c-63e3-4a22-8e8c-29255b97f933" />
      </div>
    </li>
    <li> Baseline Visible Power (5.55 uW):
      <br>The anomalies are caused by the surge in power when turning the laser back on, this effect is only an issue when the input power is high</br>
      <div style="display: flex; justify-content: space-around; align-items: center;">
        <img width="45%" alt="clipboard50" src="https://github.com/user-attachments/assets/813397b4-44dd-4ec6-b35d-819161326217" />
        <img width="45%" alt="clipboard156" src="https://github.com/user-attachments/assets/12abedaa-f247-4120-a219-16ac6c75d571" />
      </div>
    </li>
  </ul>
</p>

<p>
  <b>Noise Reduction:</b>
  <br>Here the Fast Steering Mirror (FSM) was used to correct sinosoidal noise with a frequency of 0.1 Hz and an amplitude of 2 Vpp</br>
  
  Below are the results for the Minimum Power (0.0338 uW) and then a Baseline Visible Power (5.55 uW) intially showing the pixel displacement from the origin for each axis and then distrebution of the error from the origin for each axis all for with and withought FSM turned on

For the distrebution plots the initial settling down of the tracking is ignored

  <ul>
    <li> Minimum Power (0.0338 uW):
      <div style="display: flex; justify-content: space-around; align-items: center;">
        <img width="45%" alt="clipboard2412" src="https://github.com/user-attachments/assets/315ad948-0fdc-48dd-a8ea-f677f30d4db1" />
        <img width="45%" alt="clipboard2354" src="https://github.com/user-attachments/assets/a469d13b-30c6-4d73-8141-11e6239a29ba" />
      </div>
    </li>
    <li> Baseline Visible Power (5.55 uW):
      <div style="display: flex; justify-content: space-around; align-items: center;">
        <img width="45%" alt="clipboard904" src="https://github.com/user-attachments/assets/98be8a8e-5a0c-4f81-a3a5-5032cdea626b" />
        <img width="45%" alt="clipboard1225" src="https://github.com/user-attachments/assets/b58dba47-95eb-4e4a-9473-a034a9caff2d" />
      </div>
    </li>
  </ul>
</p>

<h2>Additional Notes:</h2>
<ul>
  <li>
    The Optotune FSM takes in an Amplitude between 1.0 and -1.0 as input to move rather than an angle, to convert between angle and amplitude for this FSM the following is provided in the manual: 
    <div style="display: flex; justify-content: space-around; align-items: center;">
      <img width="27%" alt="image" src="https://github.com/user-attachments/assets/be2a6242-05cc-47bd-aa23-790d8cace5e9" />
      <img width="63%" alt="image" src="https://github.com/user-attachments/assets/401cff59-259e-460d-9b85-03c7b19b0fe0" />
    </div>
  </li>
</ul>
