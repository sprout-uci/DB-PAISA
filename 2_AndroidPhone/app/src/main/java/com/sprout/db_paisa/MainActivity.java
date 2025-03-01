package com.sprout.db_paisa;

import android.Manifest;
import android.annotation.SuppressLint;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.le.AdvertiseData;
import android.bluetooth.le.BluetoothLeAdvertiser;
import android.bluetooth.le.BluetoothLeScanner;
import android.bluetooth.le.ScanCallback;
import android.bluetooth.le.ScanFilter;
import android.bluetooth.le.ScanSettings;
import android.content.pm.PackageManager;

import android.bluetooth.BluetoothAdapter;
import android.bluetooth.le.AdvertisingSet;
import android.bluetooth.le.AdvertisingSetParameters;
import android.bluetooth.le.AdvertisingSetCallback;

import android.os.AsyncTask;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.os.ParcelUuid;
import android.util.Base64;
import android.util.Log;


import android.widget.Button;
import android.widget.TextView;

import androidx.annotation.RequiresApi;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;
import androidx.recyclerview.widget.LinearLayoutManager;
import androidx.recyclerview.widget.RecyclerView;
import androidx.recyclerview.widget.SimpleItemAnimator;

import java.io.BufferedReader;
import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.net.HttpURLConnection;
import java.net.URL;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.StandardCharsets;
import java.security.InvalidKeyException;
import java.security.KeyFactory;
import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;
import java.security.PublicKey;
import java.security.Signature;
import java.security.SignatureException;
import java.security.cert.CertificateException;
import java.security.cert.CertificateFactory;
import java.security.cert.X509Certificate;
import java.security.spec.InvalidKeySpecException;
import java.security.spec.X509EncodedKeySpec;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Date;
import java.util.Dictionary;
import java.util.HashSet;
import java.util.Hashtable;
import java.util.List;
import java.util.Locale;
import java.util.UUID;

import com.sprout.db_paisa.DBPaisaScanResult;

public class MainActivity extends AppCompatActivity {

    public String print(byte[] bytes) {
        StringBuilder sb = new StringBuilder();
        sb.append(String.format("Size: %d\n", bytes.length));
        sb.append("====================================BEGIN====================================\n");
        int i=0;
        for (byte b : bytes) {
            sb.append(String.format("[%03d] 0x%02X ", i, b));
            if ((i+1) < bytes.length && (i+1) % 10 == 0) {
                sb.append("\n");
            }
            i++;
        }
        sb.append("\n=====================================END=====================================\n");
        return sb.toString();
    }

    protected static final String TAG = "MonitoringActivity";

    private static final String LOG_TAG = "AndroidExample";

    private static final int MY_REQUEST_CODE = 123;

    private Button buttonScan;

    private TextView textViewScanResults;

    private BluetoothAdapter bluetoothAdapter;
    private BluetoothLeAdvertiser bluetoothAdvertiser;

    final AdvertisingSet[] currentAdvertisingSet = new AdvertisingSet[1];

    // ble scan code
    private BluetoothLeScanner bluetoothLeScanner; // = bluetoothAdapter.getBluetoothLeScanner();
    private ScanCallback scanCallback;
    private static boolean scanning;
    private Handler handler; // = new Handler();
    // Stops scanning after 10 seconds.
    private static final long SCAN_PERIOD = 10000;

    // results
    private ArrayList<DBPaisaScanResult> scanResults = new ArrayList<>();

    protected MC_RecyclerViewAdapter adapter;

    private HashSet<String> deviceIds = new HashSet<>();

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        // advertiser
        this.bluetoothAdapter = BluetoothAdapter.getDefaultAdapter();
        this.bluetoothAdvertiser = BluetoothAdapter.getDefaultAdapter().getBluetoothLeAdvertiser();

        // Check if all features are supported
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            if (!bluetoothAdapter.isLe2MPhySupported()) {
                Log.e(LOG_TAG, "2M PHY not supported!");
                return;
            }

            if (!bluetoothAdapter.isLeExtendedAdvertisingSupported()) {
                Log.e(LOG_TAG, "LE Extended Advertising not supported!");
                return;
            }

            int maxDataLength = bluetoothAdapter.getLeMaximumAdvertisingDataLength();
        }

        // scanner
        // check if available
        boolean bluetoothLEAvailable = getPackageManager().hasSystemFeature(PackageManager.FEATURE_BLUETOOTH_LE);

        ActivityCompat.requestPermissions(this,
                new String[]{
                        Manifest.permission.ACCESS_COARSE_LOCATION,
                        Manifest.permission.ACCESS_FINE_LOCATION,
                        Manifest.permission.ACCESS_NETWORK_STATE,
                        Manifest.permission.BLUETOOTH_ADVERTISE,
                        Manifest.permission.BLUETOOTH_SCAN,
                        Manifest.permission.BLUETOOTH_ADMIN,
                }, MY_REQUEST_CODE);


        this.scanLeDevice();

        this.buttonScan = (Button) this.findViewById(R.id.button_scan);
//        this.textViewScanResults = (TextView) this.findViewById(R.id.textView_scanResults);
        this.buttonScan.setOnClickListener(v -> askAndStartBluetoothRequest());


        RecyclerView recyclerView = findViewById(R.id.recylerview);
        ((SimpleItemAnimator) recyclerView.getItemAnimator()).setSupportsChangeAnimations(false);

        adapter = new MC_RecyclerViewAdapter(this, scanResults);
        recyclerView.setAdapter(adapter);
        recyclerView.setLayoutManager(new LinearLayoutManager(this));
    }

    private ScanCallback callback = new ScanCallback() {
        @Override
        public void onScanResult(int callbackType, android.bluetooth.le.ScanResult result) {
            byte[] scanRecord = result.getScanRecord().getBytes();

            final String DB_PAISA_ID = "DB-REQ";

            if ((new String(scanRecord, StandardCharsets.UTF_8).contains(DB_PAISA_ID))) {
                MainActivity.this.showPacketDetails(scanRecord);
            }
        }

        @Override
        public void onBatchScanResults(List<android.bluetooth.le.ScanResult> results) {
        }

        @Override
        public void onScanFailed(int errorCode) {
            Log.e("MainActivity", "lost again scan failed " + errorCode);
        }
    };

    private void scanLeDevice() {
        this.bluetoothAdapter = BluetoothAdapter.getDefaultAdapter();
        if (this.bluetoothAdapter == null || !this.bluetoothAdapter.isEnabled()) {
            Log.e(TAG, "Bluetooth adapter is not available");
        }

        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            if (!this.bluetoothAdapter.isLeExtendedAdvertisingSupported()) {
                Log.e(LOG_TAG, "LE Extended Advertising not supported!");
                return;
            } else {
                Log.i(LOG_TAG, "BLE ext adv supported!");
            }
        }

        BluetoothLeScanner bls = this.bluetoothAdapter.getBluetoothLeScanner();
        if (ActivityCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_SCAN) != PackageManager.PERMISSION_GRANTED) {
            Log.e("perms", "BLE scan perms not granted");
            return;
        }

        ScanFilter.Builder scanFilterBuilder = new ScanFilter.Builder();
        List<ScanFilter> filters = new ArrayList<>();
        filters.add(scanFilterBuilder.build());

        ScanSettings.Builder settingsBuilder = new ScanSettings.Builder();

        // works for api 26 and up
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            settingsBuilder.setLegacy(false);
            settingsBuilder.setPhy(ScanSettings.PHY_LE_ALL_SUPPORTED);
        }

        bls.startScan(filters, settingsBuilder.build(), this.callback);
    }


    private void askAndStartBluetoothRequest() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            this.advertiseBluetooth();
        }
    }

    @RequiresApi(api = Build.VERSION_CODES.O)
    void advertiseBluetooth() {
        AdvertisingSetParameters.Builder parameters = (new AdvertisingSetParameters.Builder())
                .setLegacyMode(false)
                .setInterval(AdvertisingSetParameters.INTERVAL_HIGH)
                .setTxPowerLevel(AdvertisingSetParameters.TX_POWER_MEDIUM)
                .setPrimaryPhy(BluetoothDevice.PHY_LE_1M)
                .setSecondaryPhy(BluetoothDevice.PHY_LE_2M);

        AdvertiseData data = (
                new AdvertiseData.Builder())
                .addServiceData(new ParcelUuid(UUID.randomUUID()),
                        "DP-REQ".getBytes()).build();
        AdvertisingSetCallback callback = new AdvertisingSetCallback() {
            @Override
            public void onAdvertisingSetStarted(AdvertisingSet advertisingSet, int txPower, int status) {
                Log.d(LOG_TAG, "onAdvertisingSetStarted(): txPower:" + txPower + " , status: "
                        + status);
                currentAdvertisingSet[0] = advertisingSet;
            }

            @Override
            public void onAdvertisingSetStopped(AdvertisingSet advertisingSet) {
                Log.d(LOG_TAG, "onAdvertisingSetStopped():");
            }
        };

        if (ActivityCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_ADVERTISE) != PackageManager.PERMISSION_GRANTED) {
            return;
        }
        bluetoothAdvertiser.startAdvertisingSet(parameters.build(), data, null, null, null, callback);

       new Handler(Looper.getMainLooper()).postDelayed(new Runnable() { // do we need Looper.getMainLooper ?
           @SuppressLint("MissingPermission")
           @Override
           public void run() {
               bluetoothAdvertiser.stopAdvertisingSet(callback);
           }
       }, 3000); //3s of advertising
    }

    // do we need this for bt
    @Override
    public void onRequestPermissionsResult(int requestCode, String[] permissions, int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);

        switch (requestCode) {
            case MY_REQUEST_CODE: {
                // If request is cancelled, the result arrays are empty.
                if (grantResults.length > 0 && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
                    // permission was granted
                    Log.d(LOG_TAG, "Permission Granted: " + permissions[0]);

                } else {
                    // Permission denied, boo! Disable the
                    // functionality that depends on this permission.
                    Log.d(LOG_TAG, "Permission Denied: " + permissions[0]);
                }
                break;
            }
            // Other 'case' lines to check for other
            // permissions this app might request.
        }
    }



    class DeviceProfile extends AsyncTask<String, Void, String>
    {
        private byte[] n_dev;
        private int time_cur;
        private byte[] sig;
        private byte[] attest_result;
        private int time_attest;
        private int num_req;
        private byte[] byteUrl;
        private long startTime;

        private String[] n_usr;

        @Override
        protected String doInBackground(String... params) {
            try {
                startTime = System.currentTimeMillis();

                n_dev = Base64.decode(params[0], Base64.DEFAULT);
                num_req = Integer.parseInt(params[1]);
                sig = Base64.decode(params[2], Base64.DEFAULT);
                attest_result = Base64.decode(params[3], Base64.DEFAULT);
                time_attest = Integer.parseInt(params[4]);
                System.out.println(time_attest + "time attest int");
                byteUrl = params[5].getBytes("UTF-8");
                n_usr = Arrays.copyOfRange(params, 6, params.length);

                URL url = new URL("https://bit.ly/" + params[5].substring(3));

                HttpURLConnection urlConnection = (HttpURLConnection) url.openConnection();
                urlConnection.setRequestMethod("GET");
                urlConnection.connect();

                InputStream inputStream = urlConnection.getInputStream();
                BufferedReader reader = new BufferedReader(new InputStreamReader(inputStream));

                StringBuilder stringBuilder = new StringBuilder();
                String line;
                while ((line = reader.readLine()) != null) {
                    stringBuilder.append(line + "\n");
                }

                inputStream.close();
                urlConnection.disconnect();

                return stringBuilder.toString();
            } catch (IOException e) {
                e.printStackTrace();
                return null;
            }
        }

        @Override
        protected void onPostExecute(String deviceProfile) {
            // Verification of Announced message
            StringBuilder sb = new StringBuilder();

            String[] profile = deviceProfile.split("\n");
            Dictionary<String, String> dict = new Hashtable<>();

            for (String line: profile) {
                dict.put(line.split(":", 2)[0], line.split(":", 2)[1]);
            }

            // verify signature of device profile
            byte[] profileSigBody = deviceProfile.split("signature_of_manifest")[0].replaceAll("\\\\n", "\n").replaceAll("\n\n", "\n").getBytes(StandardCharsets.UTF_8);
            String profileSig = dict.get("signature_of_manifest").replaceAll("\\\\n", "\n");
            String mfrCertString = dict.get("certificate_of_manufacturer");
            mfrCertString = mfrCertString.replaceAll("\\\\n", "\n");
            byte[] profileSigByte = Base64.decode(profileSig, Base64.DEFAULT);
            boolean isManifestSignatureValid;

            try {
                X509Certificate mfrCert = (X509Certificate) CertificateFactory.getInstance("X.509")
                        .generateCertificate(new ByteArrayInputStream(mfrCertString.getBytes(StandardCharsets.UTF_8)));
                PublicKey mfrPublicKey = mfrCert.getPublicKey();

                KeyFactory keyFactory = KeyFactory.getInstance("EC");
                X509EncodedKeySpec publicKeySpec = new X509EncodedKeySpec(mfrPublicKey.getEncoded());
                PublicKey signaturePublicKey = keyFactory.generatePublic(publicKeySpec);
                Signature signatureVerifier = Signature.getInstance("SHA256withECDSA");
                signatureVerifier.initVerify(signaturePublicKey);
                signatureVerifier.update(profileSigBody);
                isManifestSignatureValid = signatureVerifier.verify(profileSigByte);
                sb.append("Manifest Verification: " + (isManifestSignatureValid==true?"PASS":"FAIL") + "\n");
            } catch (CertificateException | InvalidKeySpecException | NoSuchAlgorithmException |
                     SignatureException | InvalidKeyException e) {
                throw new RuntimeException(e);
            }

            // verify signature of announced message from IoT device
            String devCertString = dict.get("certificate_of_device");
            devCertString = devCertString.replaceAll("\\\\n", "\n");


//            Date dateAttTs = new Date((long)ByteBuffer.wrap(time_attest).order(ByteOrder.LITTLE_ENDIAN).getInt()*1000); // convert epoch time to Date object
//
//            SimpleDateFormat dateFormat = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss", Locale.getDefault());
//            String formattedAttTs = dateFormat.format(dateAttTs); // format date as string
//            System.out.println(formattedAttTs + "time");
//            Date dateTs = new Date((long)ByteBuffer.wrap(time_cur).order(ByteOrder.LITTLE_ENDIAN).getInt()*1000); // convert epoch time to Date object
//            String formattedTs = dateFormat.format(dateTs); // format date as string


            // signature: [n_dev(32) || time_cur (4) from Dev || id_dev(4) || H(M_SRV_URL)(32) || attest_result(1) || time_attest(4)]
            X509Certificate devCert;
            boolean isSignatureValid;
            boolean attResultBool;
            try {
                MessageDigest digest = MessageDigest.getInstance("SHA-256");
                ByteArrayOutputStream baos = new ByteArrayOutputStream();

                baos.write(n_dev, 0, n_dev.length);
//                baos.write(ByteBuffer.allocate(1).putInt(num_req).array(), 0, 1);
                baos.write(num_req);
                for (int i=0; i<num_req; i++) {
                    byte[] b = Base64.decode(n_usr[i], Base64.DEFAULT);
                    baos.write(b, 0, b.length);
                }

                baos.write(byteUrl, 0, byteUrl.length);

                baos.write(attest_result, 0, attest_result.length);

                baos.write(ByteBuffer.allocate(4).order(ByteOrder.LITTLE_ENDIAN).putInt(time_attest).array(), 0, 4);

                devCert = (X509Certificate) CertificateFactory.getInstance("X.509")
                        .generateCertificate(new ByteArrayInputStream(devCertString.getBytes(StandardCharsets.UTF_8)));
                PublicKey devPublicKey = devCert.getPublicKey();
                KeyFactory keyFactory = KeyFactory.getInstance("EC");
                X509EncodedKeySpec devPublicKeySpec = new X509EncodedKeySpec(devPublicKey.getEncoded());
                PublicKey signaturePublicKey = keyFactory.generatePublic(devPublicKeySpec);
                Signature signatureVerifier = Signature.getInstance("SHA256withECDSA");
                signatureVerifier.initVerify(signaturePublicKey);
                signatureVerifier.update(baos.toByteArray());
                isSignatureValid = signatureVerifier.verify(sig);
                attResultBool = attest_result[0] == 0? true: false;

                sb.append("Announce Verification: " + (isSignatureValid==true?"PASS":"FAIL") + "\n");
                sb.append("Attestation Result: " + (attResultBool==true?"PASS":"FAIL") + "+ \" \\n\\t\\t\\t\\t\\t Attested before " + time_attest/1000 + "." + time_attest%1000/100 + "sec \n\n");
                sb.append("ID: " + dict.get("device_id")+"\n");
                sb.append("Type: " + dict.get("device_type")+"\n");
                sb.append("Manufacturer: " + dict.get("manufacturer")+"\n");
                sb.append("Status: " + dict.get("device_status")+"\n");
                sb.append("Sensors: -\n");
                sb.append("Actuators: " + dict.get("actuators")+"\n");
                sb.append("Network: " + dict.get("network")+"\n");
                sb.append("Description: " + dict.get("description")+"\n");

//                System.out.println(sb.toString() + "printing sb");
            } catch (CertificateException | InvalidKeySpecException | NoSuchAlgorithmException |
                    SignatureException | InvalidKeyException e) {
                throw new RuntimeException(e);
            }

            if (deviceIds.add(dict.get("device_id"))) {
                DBPaisaScanResult scanResult = new DBPaisaScanResult(
                        isManifestSignatureValid,
                        isSignatureValid,
                        attResultBool, time_attest/1000 + "." + time_attest%1000/100,
                        dict.get("device_id"), dict.get("device_type"), dict.get("manufacturer"),
                        dict.get("device_status"),
                        dict.get("sensors"),
                        dict.get("actuators"),
                        dict.get("network"),
                        dict.get("description"));

                // add new result to the end of the arraylist
                int insertIndex = scanResults.size();
                scanResults.add(insertIndex, scanResult);
                adapter.notifyItemInserted(insertIndex);
            }
        }
    }

    private void showPacketDetails(byte[] scanRecord) {
//        this.textViewScanResults.setText("");
        byte[] msg = {0};

        msg = Arrays.copyOfRange(scanRecord, 6, scanRecord.length);

        int msg_ptr = 0;

        String n_dev = Base64.encodeToString(
                Arrays.copyOfRange(msg, 0, 12), Base64.DEFAULT);
        msg_ptr += 12;
        int num_req = msg[msg_ptr] & 0xFF;
        msg_ptr += 1;

        String[] n_usr = new String[num_req];
        for (int i = 0; i<num_req; i++) {
            n_usr[i] = Base64.encodeToString(
                    Arrays.copyOfRange(msg, msg_ptr, msg_ptr+12), Base64.DEFAULT);
            msg_ptr += 12;
        }

        String url = new String(
                Arrays.copyOfRange(msg, msg_ptr, msg_ptr+10), StandardCharsets.UTF_8);
        msg_ptr += 10;

        String attest_result = Base64.encodeToString(
                Arrays.copyOfRange(msg, msg_ptr, msg_ptr+1), Base64.DEFAULT);
        msg_ptr += 1;

        int time_attest = ByteBuffer.wrap(
                Arrays.copyOfRange(msg, msg_ptr, msg_ptr+4)).order(ByteOrder.LITTLE_ENDIAN).getInt();
        msg_ptr += 4;

        String sig = Base64.encodeToString(
                Arrays.copyOfRange(msg, msg_ptr, msg.length), Base64.DEFAULT);

        String[] params = new String[6 + n_usr.length];
        params[0] = n_dev;
        params[1] = String.valueOf(num_req);
        params[2] = sig;
        params[3] = attest_result;
        params[4] = String.valueOf(time_attest);
        params[5] = url;
        System.arraycopy(n_usr, 0, params, 6, n_usr.length);

        DeviceProfile deviceProfile = new DeviceProfile();
        deviceProfile.executeOnExecutor(AsyncTask.THREAD_POOL_EXECUTOR, params);
    }

}