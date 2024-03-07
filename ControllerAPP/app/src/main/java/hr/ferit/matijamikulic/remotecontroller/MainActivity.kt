package hr.ferit.matijamikulic.remotecontroller

import android.Manifest
import android.annotation.SuppressLint
import android.app.Activity
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothManager
import android.content.Context
import android.content.Intent
import android.content.pm.PackageManager
import android.os.*
import androidx.appcompat.app.AppCompatActivity
import android.util.Log
import android.view.MotionEvent
import androidx.activity.result.contract.ActivityResultContracts
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.launch
import java.io.IOException
import java.nio.charset.Charset
import java.util.Timer

import android.widget.*

class MainActivity : AppCompatActivity() {

    companion object{
        const val DEBUG_TAG = "BL CONNECTION"
        //esp32  BLUETOOTH DEVICE
        const val MAC_ADDRESS = "7C:87:CE:30:79:56"
        const val UUID = "00001101-0000-1000-8000-00805f9b34fb"
        const val REQUEST_CODE_BLUETOOTH = 1000

        //HC-06 BLUETOOTH DEVICE
        //const val MAC_ADDRESS = "98:DA:60:04:F8:EA"
        //const val UUID = "00001101-0000-1000-8000-00805f9b34fb"
    }
    //region Bluetooth Related
    private lateinit var bluetoothManager: BluetoothManager
    private lateinit var bluetoothAdapter: BluetoothAdapter
    private lateinit var device: BluetoothDevice
    private lateinit var connectBtn: Button
    private lateinit var disconnectBtn: Button
    private var bluetoothService:BluetoothService? = null
    //endregion

    //region Timer Related
    private var timer: Timer? = null
    private val UPDATE_INTERVAL: Long = 250
    @Volatile
    private var receivedData: String? = null
    //endregion

    //region UI
    private lateinit var forwardButton: Button
    private lateinit var backwardButton: Button
    private lateinit var rightButton: Button
    private lateinit var leftButton: Button
    private lateinit var stopButton: Button
    private lateinit var yawTxtView : TextView
    private lateinit var bSpeed: TextView
    private lateinit var forwardEditText : EditText
    private lateinit var BackwardEditText : EditText
    private lateinit var RightEditText : EditText
    private lateinit var LeftEditText : EditText
    private lateinit var appendButton : Button
    private lateinit var executeButton: Button
    private lateinit var undoButton: Button

    //endregion

    private var commandString="";
    private val commandList = mutableListOf<String>()


    //handling received and sent data
    private val handler = object : Handler(Looper.getMainLooper()) {
        override fun handleMessage(msg: Message) {
            when (msg.what) {
                MESSAGE_READ -> {
                    val numBytes = msg.arg1
                    if(numBytes >0){
                        val readBuf = msg.obj as ByteArray
                        val readMsg = readBuf.copyOfRange(0, numBytes).toString(Charset.defaultCharset())
                        val values = readMsg.split(",")
                        if (values.size >= 3) {
                            val intValue1 = values[0]
                            val intValue2 = values[1]
                            val intValue3 = values[2]
                            // Update UI with the latest data
                            updateUI(intValue1, intValue2, intValue3)
                        }
                    }
                }
                MESSAGE_WRITE -> {
                    val writeBuf = msg.obj as ByteArray
                    // Handle outgoing message
                }
                MESSAGE_TOAST -> {
                    val toastMsg = msg.data.getString("toast")
                    Toast.makeText(applicationContext, toastMsg, Toast.LENGTH_SHORT).show()
                }
            }
        }
    }

    private fun updateUI(intValue1: String, intValue2: String, intValue3: String) {
        yawTxtView.text = intValue3
        bSpeed.text = intValue2
    }

    private fun saveDataToFile(data: String) {
        try {
            val fileName = "received_data.txt"
            val fileContents = "$data\n" // Append a new line
            applicationContext.openFileOutput(fileName, Context.MODE_APPEND).use {
                it.write(fileContents.toByteArray())
            }
            Log.d(DEBUG_TAG, "Data saved to file")
        } catch (e: IOException) {
            Log.e(DEBUG_TAG, "Error saving data to file", e)
        }
    }


    private fun connectDevices() {
        CoroutineScope(Dispatchers.IO).launch {
                try {
                    // Get the BluetoothDevice object for the remote device.
                    device = bluetoothAdapter.getRemoteDevice(MAC_ADDRESS)

                    // Create a BluetoothSocket object.
                    val socket = device.createRfcommSocketToServiceRecord(java.util.UUID.fromString(UUID))
                    bluetoothAdapter.cancelDiscovery()

                    // Try to connect to the Bluetooth device.
                    socket.connect()

                    // Create a BluetoothService object to handle the communication with the Bluetooth device.
                    // Start the communication with the Bluetooth device.
                    bluetoothService = BluetoothService(handler)
                    bluetoothService?.startCommunication(socket!!)

                } catch (e: IOException) {
                    Log.e(DEBUG_TAG, "Could not connect to device ${device.name}", e)
                }
        }
    }

    //checking if android device has bluetooth enabled, if not aks permission to enable it
    private val enableBTLauncher = registerForActivityResult(
        ActivityResultContracts.StartActivityForResult()
    ) { result ->
        if (result.resultCode == Activity.RESULT_OK) {
            val permissions = arrayOf(Manifest.permission.BLUETOOTH)
            requestPermissions(permissions, REQUEST_CODE_BLUETOOTH)
        } else {
            Log.d(DEBUG_TAG, "Cannot connect to bluetooth.")
            Toast.makeText(this, "Cannot connect to bluetooth.", Toast.LENGTH_SHORT).show()
            return@registerForActivityResult
        }
    }

    override fun onRequestPermissionsResult(
        requestCode: Int,
        permissions: Array<out String>,
        grantResults: IntArray
    ) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults)

        if (requestCode == REQUEST_CODE_BLUETOOTH && grantResults.isNotEmpty() && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
            connectDevices()
        } else {
            Log.d(DEBUG_TAG, "Cannot connect to bluetooth.")
            Toast.makeText(this, "Cannot connect to bluetooth.", Toast.LENGTH_SHORT).show()
        }
    }

    private var requestBluetooth = registerForActivityResult(ActivityResultContracts.StartActivityForResult()) { result ->
        if (result.resultCode == RESULT_OK) {
            // granted
            Log.d(DEBUG_TAG,"Granted access")
            // ask to turn on bluetooth
            if(!bluetoothAdapter.isEnabled){
                val enableBTIntent = Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE)
                enableBTLauncher.launch(enableBTIntent)
            }
        }else{
            //deny
            Log.d(DEBUG_TAG,"No bluetooth access")
        }
    }

    private val requestMultiplePermissions =
        registerForActivityResult(ActivityResultContracts.RequestMultiplePermissions()) { permissions ->
            permissions.entries.forEach {
                Log.d("test006", "${it.key} = ${it.value}")
            }
        }

    @SuppressLint("ClickableViewAccessibility")
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        initComponents()

        //Ask for bluetooth permission
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            requestMultiplePermissions.launch(arrayOf(
                Manifest.permission.BLUETOOTH_SCAN,
                Manifest.permission.BLUETOOTH_CONNECT))
        }
        else{
            val enableBtIntent = Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE)
            requestBluetooth.launch(enableBtIntent)
        }
        //Ask to turn on bluetooth
        if(!bluetoothAdapter.isEnabled){
            val enableBTIntent = Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE)
            enableBTLauncher.launch(enableBTIntent)

        }else{
            connectDevices()
        }

        //region Button Event Listeners
        connectBtn.setOnClickListener {
            connectDevices()
        }
        disconnectBtn.setOnClickListener {
            bluetoothService?.cancel()
        }

        forwardButton.setOnTouchListener { _, event ->
            when (event.action) {
                MotionEvent.ACTION_DOWN -> {
                    bluetoothService?.write("&w;".toByteArray());
                }
                MotionEvent.ACTION_UP -> {
                    // Add delay before closing socket
                    bluetoothService?.write("&x;".toByteArray());
                }
            }
            true
        }
        backwardButton.setOnTouchListener { _, event ->
            when (event.action) {
                MotionEvent.ACTION_DOWN -> {
                    bluetoothService?.write("&s;".toByteArray());
                }
                MotionEvent.ACTION_UP -> {
                    bluetoothService?.write("&x;".toByteArray());
                }
            }
            true
        }

        rightButton.setOnTouchListener { _, event ->
            when (event.action) {
                MotionEvent.ACTION_DOWN -> {
                    bluetoothService?.write("&d;".toByteArray());
                }
                MotionEvent.ACTION_UP -> {
                    bluetoothService?.write("&x;".toByteArray());
                }
            }
            true
        }

        leftButton.setOnTouchListener { _, event ->
            when (event.action) {
                MotionEvent.ACTION_DOWN -> {
                    bluetoothService?.write("&a;".toByteArray());
                }
                MotionEvent.ACTION_UP -> {
                    bluetoothService?.write("&x;".toByteArray());
                }
            }
            true
        }

        stopButton.setOnClickListener {
            bluetoothService?.write("&x;".toByteArray());
        }

        //endregion
        appendButton.setOnClickListener {
            if(!forwardEditText.text.isEmpty()){
                commandList.add("&w${forwardEditText.text};")
                forwardEditText.text.clear()
            }
            if(!RightEditText.text.isEmpty()){
                commandList.add("&d${RightEditText.text};")
                RightEditText.text.clear()
            }
            if(!LeftEditText.text.isEmpty()){
                commandList.add("&a${LeftEditText.text};")
                LeftEditText.text.clear()
            }
            if(!BackwardEditText.text.isEmpty()){
                commandList.add("&s${BackwardEditText.text};")
                BackwardEditText.text.clear()
            }

        }
        undoButton.setOnClickListener {
            if (commandList.isNotEmpty()) {
                // Remove the last added command from the list
                commandList.removeAt(commandList.size - 1)
            }
        }

        executeButton.setOnClickListener {
            if (commandList.isNotEmpty()) {
                val commandString = commandList.joinToString(separator = "")
                bluetoothService?.write(commandString.toByteArray())
                commandList.clear()
            }
        }

    }

    private fun initComponents(){
        bluetoothManager = getSystemService(Context.BLUETOOTH_SERVICE) as BluetoothManager
        bluetoothAdapter = bluetoothManager.adapter

        connectBtn = findViewById(R.id.connect)
        disconnectBtn = findViewById(R.id.disconnect_btn)
        forwardButton = findViewById(R.id.forward_btn)
        backwardButton = findViewById(R.id.backward_btn)
        rightButton = findViewById(R.id.right_btn)
        leftButton = findViewById(R.id.left_btn)
        stopButton = findViewById(R.id.stop_btn)
        yawTxtView = findViewById(R.id.yawAngle_txtView)
        bSpeed = findViewById(R.id.bSpeed_txtView)

        forwardEditText = findViewById(R.id.forwardET)
        BackwardEditText = findViewById(R.id.backwardET)
        RightEditText = findViewById(R.id.rightET)
        LeftEditText = findViewById(R.id.leftET)

        appendButton = findViewById(R.id.button)
        executeButton = findViewById(R.id.button2)
        undoButton = findViewById(R.id.button3)

    }

    private fun getSelectedEditText(): EditText? {
        val selectedId = resources.getResourceEntryName(resources.getIdentifier("selected_edit_text", "id", packageName))
        return findViewById(resources.getIdentifier(selectedId, "id", packageName))
    }

    private fun getCommandPrefix(editText: EditText): String {
        val id = editText.id
        return when (id) {
            R.id.forwardET -> "&w"
            R.id.rightET -> "&d"
            R.id.leftET -> "&a"
            else -> throw IllegalArgumentException("Invalid EditText ID for command prefix")
        }
    }


    //close socket
    override fun onDestroy() {
        super.onDestroy()
        bluetoothService?.cancel()
    }
}