<h1 align="center">Satelite Tracking</h1>

<h2>Description:</h2>

<p>
</p>

<h2>Usage:</h2>
  <b>Camera Setup</b>:
  <ol>
    <li>Visit www.zwoastro.com/software</li>
    <li>
      Download and install ASIStudio and the appropriate camera driver, the ASICap software is useful for fiddling around with settings
    </li>
    <li>On the same page, got to the "Others" tab</li>
    <li>Locate and download the ASI Camera SDK from develop tools</li>
    <li>Interface with the SDK using the ZWOASI Python library available here https://github.com/python-zwoasi</li>
  </ol>
  <b>Optotune FSM Setup:</b>
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
  <b>Code Setup:</b>
  <ol>
    <li>Activate a virtual environment.</li>
    <li>Run <code>pip install -r requirements.txt</code> to install the dependencies.</li>
    <li>Run <code>main.py</code> to either tune the kalman filter or start tracking.</li>
  </ol>  

  <h2>Program Overview</h2>

  <h3>lazer_tracking.py</h3>

