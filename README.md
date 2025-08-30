<h1 align="center">Satellite Tracking</h1>

<h2>Description:</h2>
<p>
This repository holds the foundational code for a satellite tracking and beam stabilization system.
</p>

<h3>Features</h3>
<ul>
  <li>Hardware control with ZWO ASI cameras and Optotune FSM controllers</li>
  <li>Fast Steering Mirror (FSM) noise reduction</li>
  <li>Constant-velocity Kalman Filter tracking</li>
  <li>Kalman Filter parameter tuning</li>
  <li>System setup and calibration tools</li>
  <li>FFT-based noise frequency identification</li>
</ul>

<h3>Status</h3>
<p>
  The program includes a tracking and noise reduction loop, but it has not yet been fully tested or tuned.  
  At this stage, the loop represents my initial design of how tracking and noise reduction could operate together.  
  To allow for flexibility, the Kalman Filter can be toggled on or off so that noise reduction can be evaluated independently of tracking.  
  The FFT-based noise frequency identification is not yet integrated into the loop, but the core code for this functionality is already implemented.
</p>

<h2>Setup:</h2>
  <h3>Camera:</h3>
  <ol>
    <li>Visit www.zwoastro.com/software</li>
    <li>
      Download and install ASIStudio and the appropriate camera driver. The ASICap software is useful for adjusting settings.
    </li>
    <li>On the same page, go to the "Others" tab</li>
    <li>Locate and download the ASI Camera SDK from the developer tools</li>
    <li>Interface with the SDK using the ZWOASI Python library available here: https://github.com/python-zwoasi</li>
  </ol>
  <h3>Optotune FSM:</h3>
  <ol>
    <li>Check the computer detects the input device, e.g. <code>ls /dev/cu.*</code> (Mac terminal)</li>
    <li>Connect the Fast Steering Mirror to the controller</li>
    <li>Download a hyperterminal to interact with the controller, e.g. CoolTerm</li>
    <li>Check the terminal settings match the following:
      <ul>
        <li>Baud Rate: 25600</li>
        <li>Data Bits: 8</li>
        <li>Stop Bits: 1</li>
        <li>Parity: None </li>
      </ul>  
    </li>
    <li>Make sure “Local echo” is enabled and “Enter Key Emulation” is set to CR + LF</li>
    <li>Send ‘start’ to check the FSM is connected; <code>OK</code> should be returned</li>
    <li>Refer to <code>optotune_fsm_manual</code> found in the <code>hardware_manuals</code> folder in this shared drive for more information</li>
  </ol>
  <h3>Code:</h3>
  <ol>
    <li>Activate a virtual environment.</li>
    <li>Run <code>pip install -r requirements.txt</code> to install the dependencies.</li>
    <li>Edit the input parameters in the <code>config.yaml</code> file</li>
    <li>Run <code>main.py</code> to either tune the Kalman filter or start tracking.</li>
  </ol>  
  <h3>Model:</h3>
  
  <div style="display: flex; justify-content: space-around; align-items: center;">
    <img width="42%" alt="image" src="https://github.com/user-attachments/assets/0e07f7a0-8345-44b4-8b4e-0538e31a8ccd" />
    <img width="48%" alt="image" src="https://github.com/user-attachments/assets/11cfd4fe-7222-4ce4-9943-d40618789b47" />
  </div>



<h2>Results:</h2>

<p>
  All raw result CSV files can be found in the <code>results</code> directory.
</p>
<p>
  <h3>Image noise reduction:</h3>

  The images below show the following:
  <ul>
    <li>Left: Raw frame</li>
    <li>Middle: Raw frame with dark frame subtraction applied</li>
    <li>Right: Raw frame with dark frame subtraction and morphological opening applied</li>
  </ul>

  <img width="1101" height="320" alt="image" src="https://github.com/user-attachments/assets/c764d708-3679-44e9-9f8a-3926cff53792" />
</p>

<p>
  <h3>Identifying laser position:</h3>

  The images below show the following:
  <ol>
    <li>Image 1: Clean frame</li>
    <li>Image 2: Clean frame with contours outlined</li>
    <li>Image 3: Clean frame with the largest contour outlined and its center pinpointed </li>
  </ol>
  
  <img width="1428" height="352" alt="image" src="https://github.com/user-attachments/assets/a3916388-4a81-497e-8e90-a6e29302c978" />
</p>

<p>
  <h3>Finding the minimum laser power at which the laser could be identified:</h3>
  <ul>
    <li>
      For each laser power I measured the area, mean intensity, and standard deviation intensity. I also evaluated how well the laser could reproduce input noise (a square wave with a frequency of 1 Hz). To do this, I calculated the dominant frequency of the FFT of the pixel position over time. The frequency error plotted against laser power can also be seen below.
      <img width="1389" height="990" alt="clipboard706" src="https://github.com/user-attachments/assets/980676aa-693e-42d9-99af-b96c717495d4" />
    </li>
    <li>
      Since the mean intensity and standard deviation of the detected laser depend on the beam profile’s measured area (as higher laser intensity causes the profile to “bloom” and include more of the faint halo), these raw metrics become misleading: the mean is artificially lowered, and the standard deviation inflated.
      <p>
        To remove this area dependency, two normalized quantities were introduced and plotted against the laser current:
        <ul>
          <li>
            Total Flux (Mean Intensity × Area): Provides a true measure of the total captured light energy, independent of beam size.
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
  <h3>Tracking the laser:</h3> 
  Here the FSM was used to move the laser beam at a constant velocity across the frame to simulate satellite movement. Occasionally the beam was turned off to simulate an obstruction (e.g. a cloud passing overhead), allowing the Kalman Filter to estimate the subsequent positions of the laser (satellite).

  Below are the results for the Minimum Power (0.0338 µW) and then a Baseline Visible Power (5.55 µW). These initially show the X position plotted against the Y position, and then the X and Y positions individually plotted against time, all for both the estimated (Kalman Filter ON) and measured (Kalman Filter OFF) values.

  <ul>
    <li> Minimum Power (0.0338 µW):
      <div style="display: flex; justify-content: space-around; align-items: center;">
        <img width="45%" alt="clipboard2075" src="https://github.com/user-attachments/assets/21ec492d-bdf9-4c0e-af64-f46842a4fbb2" />
        <img width="45%" alt="clipboard1165" src="https://github.com/user-attachments/assets/6f4cff0c-63e3-4a22-8e8c-29255b97f933" />
      </div>
    </li>
    <li> Baseline Visible Power (5.55 µW):
      <br>The anomalies are caused by the surge in power when turning the laser back on. This effect is only an issue when the input power is high.</br>
      <div style="display: flex; justify-content: space-around; align-items: center;">
        <img width="45%" alt="clipboard50" src="https://github.com/user-attachments/assets/813397b4-44dd-4ec6-b35d-819161326217" />
        <img width="45%" alt="clipboard156" src="https://github.com/user-attachments/assets/12abedaa-f247-4120-a219-16ac6c75d571" />
      </div>
    </li>
  </ul>
</p>

<p>
  <h3>Noise Reduction:</h3>
  Here the Fast Steering Mirror (FSM) was used to correct sinusoidal noise with a frequency of 0.1 Hz and an amplitude of 2 Vpp.
  
  Below are the results for the Minimum Power (0.0338 µW) and then a Baseline Visible Power (5.55 µW). These initially show the pixel displacement from the origin for each axis, and then the distribution of the error from the origin for each axis, both with and without the FSM turned on.

For the distribution plots, the initial settling of the tracking is ignored.

  <ul>
    <li> Minimum Power (0.0338 µW):
      <div style="display: flex; justify-content: space-around; align-items: center;">
        <img width="45%" alt="clipboard2412" src="https://github.com/user-attachments/assets/315ad948-0fdc-48dd-a8ea-f677f30d4db1" />
        <img width="45%" alt="clipboard2354" src="https://github.com/user-attachments/assets/a469d13b-30c6-4d73-8141-11e6239a29ba" />
      </div>
    </li>
    <li> Baseline Visible Power (5.55 µW):
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
    The Optotune FSM takes in an amplitude between 1.0 and -1.0 as input rather than an angle. To convert between angle and amplitude for this FSM, the following is provided in the manual: 
    <div style="display: flex; justify-content: space-around; align-items: center;">
      <img width="27%" alt="image" src="https://github.com/user-attachments/assets/be2a6242-05cc-47bd-aa23-790d8cace5e9" />
      <img width="63%" alt="image" src="https://github.com/user-attachments/assets/401cff59-259e-460d-9b85-03c7b19b0fe0" />
    </div>
  </li>
</ul>
