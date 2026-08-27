# SystemCore / Driver Station Troubleshooting

## Test in This Order

### 1. Verify SystemCore Version

* [ ] Open the SystemCore web interface.
* [ ] Record the exact SystemCore image/version.
* [ ] Confirm the installed version is compatible with the 2027 Driver Station.

### 2. Test SystemCore Directly

* [ ] Connect the Driver Station computer directly to SystemCore.
* [ ] Open `http://10.17.78.2`.
* [ ] Confirm the SystemCore web interface loads.

### 3. Test USB Networking

* [ ] Connect through USB-C.
* [ ] Run `ipconfig /all`.
* [ ] Identify the SystemCore USB network adapter.
* [ ] Test the current SystemCore USB address.
* [ ] Confirm whether the SystemCore is reachable over USB.

### 4. Deploy With Full Logging

Run:

```powershell
.\gradlew clean deploy --info
```

* [ ] Save the complete terminal output.
* [ ] Record the SystemCore host/address Gradle connects to.
* [ ] Record any deployment or startup errors.

### 5. Check Whether Robot Code Actually Starts

After deployment:

* [ ] Determine whether the program starts, crashes, or exits.
* [ ] Check the SystemCore status/interface for the robot process.
* [ ] Check robot/application logs.