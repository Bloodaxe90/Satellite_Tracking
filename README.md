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
  <li>Changing-acceleration Kalman Filter tracking (changing_acceleration_kf branch)</li>
  <li>Kalman Filter parameter tuning</li>
  <li>System setup and calibration tools</li>
  <li>FFT-based noise frequency identification</li>
</ul>

<h3>Status</h3>
<p>
The project currently features a functional prototype of the combined tracking and noise reduction loop. While individual components (Kalman Filter and FSM Control) are implemented, the system requires further tuning and integration testing.
</p>
<h4>Known Issues & Limitations</h4>
<ul>
<li>
<strong>Control Loop Conflict:</strong> There is a known conflict when running tracking and stabilization simultaneously. The noise reduction (FSM control) successfully centers the laser, which causes the Kalman Filter to perceive the target as stationary. This effectively masks the satellite's trajectory from the filter. Resolving this would be the main next step.
</li>
<li>
<strong>Static Tuning Parameters:</strong> The optimal tuning parameters for the Kalman Filter's Process Noise ($Q$) and Measurement Noise ($R$) were found to be different for continuous tracking vs. tracking with intermittent or long signal breaks. A single, static set of parameters is a compromise and may not be optimal for all scenarios.
</li>
<li>
<strong>Hard-coded Modules:</strong> The Noise Reduction (FSM control) is currently hard-coded as "active" in the main loop. To evaluate tracking independently, please use the scripts in the <code>test.py</code> file.
</li>
</ul>
<h4>Future Work & Proposed Solutions</h4>
<ul>
<li>
<strong>Dynamic Model Switching:</strong> To address the issue of static tuning parameters, an adaptive filtering approach could be implemented. This would involve either dynamically adjusting the $Q$ and $R$ values based on the current scenario, or switching between discrete filter models (e.g. a "high-certainty" model for active tracking and a "high-uncertainty" model for prediction over breaks).
</li>
<li>
<strong>Intelligent Re-acquisition Logic:</strong> A potential failure exists where the filter, after a long time of prediction, becomes overconfident in its prediction and rejects the valid re-appearance of the satellite. A more sophisticated re-acquisition algorithm is needed. This could involve temporarily expanding the filter's validation gate after a signal loss or using a secondary, simpler detection algorithm to alert the main filter that the target is back in view.
</li>
<li>
<strong>FFT Integration:</strong> The Fast Fourier Transform (FFT) logic for frequency analysis in use in a local ocillator is fully implemented but has not yet been integrated into the live control loop.
</li>
</ul>

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
    <li>Refer to <code>optotune_fsm_manual</code> found in the <code>resources/hardware_manuals</code> directory for more information</li>
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
  <img width="1389" height="990" alt="clipboard" src="https://github.com/user-attachments/assets/b2a5d593-e5e7-48cd-9fdd-6f1929eafe43" />
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
<img width="1390" height="490" alt="clipboard1" src="https://github.com/user-attachments/assets/8cbb0788-888a-4fe5-b456-1fd183bf6dac" />
      </p>
    </li>
  </ul>
</p>

<p>
  <h3>Tracking the laser: (Constant Velocity)</h3> 
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
  <h3>Tracking the laser: (Changing Acceleration)</h3> 
  Here the FSM was used to move the laser beam at a changing acceleration accelerating (accelerating towards the center of the frame and then decelerating away it) to represent the doplar effect within the simulated satellites movement. Occasionally the beam was turned off to simulate an obstruction (e.g. a cloud passing overhead), allowing the Kalman Filter to estimate the subsequent positions of the laser (satellite), in some runs noise was also added.

  Below are the results for the Minimum Power (0.0338 µW) and then a Baseline Visible Power (5.55 µW). These initially show the X position plotted against the Y position, and then the X and Y positions individually plotted against time, all for both the estimated (Kalman Filter ON) and measured (Kalman Filter OFF) values.

  <ul>
    <li> Minimum Power (0.0338 µW):
      <div style="display: flex; justify-content: space-around; align-items: center;">
        <img width="45%" alt="clipboard8" src="https://github.com/user-attachments/assets/a68d433f-dabd-4af8-a1b1-e3364352eea9" />
        <img width="45%" alt="clipboard3" src="https://github.com/user-attachments/assets/9e694b96-3c64-49d7-9537-a167860edf28" />
      </div>
    </li>
    <li> Baseline Visible Power (5.55 µW):
      <br>The anomalies are caused by the surge in power when turning the laser back on. This effect is only an issue when the input power is high.</br>
      <div style="display: flex; justify-content: space-around; align-items: center;">
        <img width="45%" alt="clipboard50" src="https://github.com/user-attachments/assets/d4521dbe-d055-4633-a747-05ff9a4ebf83" />
        <img width="45%" alt="clipboard5" src="https://github.com/user-attachments/assets/6689b376-03c8-448f-87af-f5caf90c90a6" />
      </div>
    </li>
  </ul>
  <ul>
  <li> Minimum Power (Noise) (0.0338 µW):
    <br>There is no noise along the y axis due to the noise generator only moving the laser along the X-axis</br>
    <div style="display: flex; justify-content: space-around; align-items: center;">
      <img width="45%" alt="clipboard11" src="https://github.com/user-attachments/assets/a70419df-f36f-4393-82d2-60838219f491" />
      <img width="45%" alt="clipboard1622" src="https://github.com/user-attachments/assets/e421d91a-5112-42a1-ad02-b2d99154ecc4" />
    </div>
  </li>
  <li> Baseline Visible Power (Noise) (5.55 µW):
    <br>The anomalies are caused by the surge in power when turning the laser back on. This effect is only an issue when the input power is high and there is no noise along the y axis due to the noise generator only moving the laser along the X-axis</br>
    <div style="display: flex; justify-content: space-around; align-items: center;">
      <img width="45%" alt="clipboard1458" src="https://github.com/user-attachments/assets/3de97528-43ab-4cbf-91c6-d873fcd88dc9" />
      <img width="45%" alt="clipboard4" src="https://github.com/user-attachments/assets/cc8e6d5b-a88c-4e16-9939-8c186d3f8350" />
    </div>
  </li>
  </ul>
  <ul>
  <li> Minimum Power (large break) (0.0338 µW):
    <div style="display: flex; justify-content: space-around; align-items: center;">
      <img width="45%" alt="clipboard2068" src="https://github.com/user-attachments/assets/2ad6a205-4c7d-4ea6-9f0e-3bc406bc9789" />
      <img width="45%" alt="clipboard1265" src="https://github.com/user-attachments/assets/1a24c2c9-6b9e-487c-b150-1515bfa8c424" />
    </div>
  </li>
  <li> Baseline Visible Power (large break) (5.55 µW):
    <div style="display: flex; justify-content: space-around; align-items: center;">
      <img width="45%" alt="clipboard1114" src="https://github.com/user-attachments/assets/dc2b4703-9a21-4f8a-8c9f-6e8a7d76addc" />
      <img width="45%" alt="clipboard2299" src="https://github.com/user-attachments/assets/cbfc0c76-33d2-42a3-b535-a65b7a041582" />
    </div>
  </li>
  </ul>
    <ul>
  <li> Minimum Power (large break & noise) (0.0338 µW):
      <br>
      There is no noise along the Y-axis because the noise generator only affects motion along the X-axis. The failure of the Y-axis to track correctly, while the X-axis tracks as expected, is due to the X–Y offset in the model (moving the laser along the X-axis causes a slight change in the Y-axis)
      </br>
    <div style="display: flex; justify-content: space-around; align-items: center;">
      <img width="45%" alt="clipboard2017" src="https://github.com/user-attachments/assets/7a1a79af-097b-4110-b2a2-b7ffd6854236" />
      <img width="45%" alt="clipboard1143" src="https://github.com/user-attachments/assets/78d241fa-e44a-492a-bd1c-fd12de803c15" />
    </div>
  </li>
  <li> Baseline Visible Power (large break & noise) (5.55 µW):
      There is no noise along the Y-axis because the noise generator only affects motion along the X-axis. The failure of the Y-axis to track correctly, while the X-axis tracks as expected, is due to the X–Y offset in the model (moving the laser along the X-axis causes a slight change in the Y-axis)
    <div style="display: flex; justify-content: space-around; align-items: center;">
      <img width="45%" alt="clipboard2225" src="https://github.com/user-attachments/assets/901e9b8c-0a12-431f-bf8a-7bf47cec1289" />
      <img width="45%" alt="clipboard2212" src="https://github.com/user-attachments/assets/f67c5bd4-7893-46ed-a22c-cb50c8af494c" />
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
