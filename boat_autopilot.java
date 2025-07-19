public class BoatAutopilot extends Activity implements LocationListener {
    private LocationManager locationManager;
    private UsbSerialPort serialPort;
    private List<LatLng> waypoints;
    private int currentWaypointIndex = 0;
    private boolean autopilotEnabled = false;
    
    // PID контроллер для управления курсом
    private PIDController headingPID;
    
    // Текущее состояние лодки
    private Location currentLocation;
    private float currentHeading;
    private float targetHeading;
    
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        
        initializeGPS();
        initializeUSBConnection();
        initializePIDController();
        setupWaypoints();
    }
    
    private void initializeGPS() {
        locationManager = (LocationManager) getSystemService(Context.LOCATION_SERVICE);
        
        // Запрос разрешений
        if (ActivityCompat.checkSelfPermission(this, 
            Manifest.permission.ACCESS_FINE_LOCATION) == PackageManager.PERMISSION_GRANTED) {
            
            // Включение GPS с высокой точностью
            locationManager.requestLocationUpdates(
                LocationManager.GPS_PROVIDER, 
                1000, // обновление каждую секунду
                1,    // минимальное расстояние 1 метр
                this
            );
        }
    }
    
    private void initializeUSBConnection() {
        UsbManager manager = (UsbManager) getSystemService(Context.USB_SERVICE);
        List<UsbSerialDriver> drivers = UsbSerialProber.getDefaultProber()
            .findAllDrivers(manager);
            
        if (!drivers.isEmpty()) {
            UsbSerialDriver driver = drivers.get(0);
            UsbDeviceConnection connection = manager.openDevice(driver.getDevice());
            serialPort = driver.getPorts().get(0);
            
            try {
                serialPort.open(connection);
                serialPort.setParameters(9600, 8, 
                    UsbSerialPort.STOPBITS_1, UsbSerialPort.PARITY_NONE);
                Log.i("USB", "Соединение с Arduino установлено");
            } catch (IOException e) {
                Log.e("USB", "Ошибка подключения", e);
            }
        }
    }
    
    private void initializePIDController() {
        // PID настройки для управления курсом
        double kp = 2.0;  // пропорциональный коэффициент
        double ki = 0.1;  // интегральный коэффициент  
        double kd = 0.5;  // дифференциальный коэффициент
        
        headingPID = new PIDController(kp, ki, kd);
        headingPID.setOutputLimits(-100, 100); // ограничение выхода
    }
    
    private void setupWaypoints() {
        waypoints = new ArrayList<>();
        // Пример маршрута (широта, долгота)
        waypoints.add(new LatLng(55.7558, 37.6176)); // Красная площадь
        waypoints.add(new LatLng(55.7539, 37.6208)); // ГУМ
        waypoints.add(new LatLng(55.7520, 37.6175)); // Васильевский спуск
        // Добавить остальные точки маршрута
    }
    
    @Override
    public void onLocationChanged(Location location) {
        currentLocation = location;
        
        // Отправка координат на сервер
        sendLocationToServer(location);
        
        // Автопилот
        if (autopilotEnabled) {
            updateAutopilot();
        }
        
        // Обновление UI
        updateLocationDisplay(location);
    }
    
    private void updateAutopilot() {
        if (currentWaypointIndex >= waypoints.size()) {
            // Маршрут завершен
            stopAutopilot();
            return;
        }
        
        LatLng targetWaypoint = waypoints.get(currentWaypointIndex);
        
        // Расчет расстояния до целевой точки
        float[] results = new float[1];
        Location.distanceBetween(
            currentLocation.getLatitude(), currentLocation.getLongitude(),
            targetWaypoint.latitude, targetWaypoint.longitude,
            results
        );
        
        float distanceToWaypoint = results[0];
        
        // Если достигли точки (в пределах 5 метров)
        if (distanceToWaypoint < 5.0f) {
            currentWaypointIndex++;
            Log.i("Autopilot", "Достигнута точка " + currentWaypointIndex);
            return;
        }
        
        // Расчет целевого курса
        targetHeading = calculateBearing(currentLocation, targetWaypoint);
        
        // Получение текущего курса
        currentHeading = getCurrentHeading();
        
        // PID регулирование
        double headingError = normalizeAngle(targetHeading - currentHeading);
        double rudderOutput = headingPID.calculate(headingError);
        
        // Отправка команд управления
        sendMotorCommand(80); // постоянная скорость
        sendRudderCommand((int) rudderOutput);
        
        Log.i("Autopilot", String.format(
            "Курс: %.1f°, Цель: %.1f°, Руль: %.1f", 
            currentHeading, targetHeading, rudderOutput
        ));
    }
    
    private float calculateBearing(Location from, LatLng to) {
        double lat1 = Math.toRadians(from.getLatitude());
        double lon1 = Math.toRadians(from.getLongitude());
        double lat2 = Math.toRadians(to.latitude);
        double lon2 = Math.toRadians(to.longitude);
        
        double dLon = lon2 - lon1;
        
        double y = Math.sin(dLon) * Math.cos(lat2);
        double x = Math.cos(lat1) * Math.sin(lat2) - 
                   Math.sin(lat1) * Math.cos(lat2) * Math.cos(dLon);
        
        double bearing = Math.toDegrees(Math.atan2(y, x));
        return (float) ((bearing + 360) % 360);
    }
    
    private float getCurrentHeading() {
        // Используем встроенный компас
        SensorManager sensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
        Sensor magnetometer = sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
        Sensor accelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        
        // Упрощенно - в реальной реализации нужна фильтрация
        return currentLocation.getBearing();
    }
    
    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }
    
    private void sendMotorCommand(int speed) {
        String command = "MOTOR:" + speed + "\n";
        sendSerialCommand(command);
    }
    
    private void sendRudderCommand(int angle) {
        // Ограничение угла руля
        angle = Math.max(-45, Math.min(45, angle));
        String command = "RUDDER:" + angle + "\n";
        sendSerialCommand(command);
    }
    
    private void sendSerialCommand(String command) {
        if (serialPort != null) {
            try {
                serialPort.write(command.getBytes(), 1000);
            } catch (IOException e) {
                Log.e("Serial", "Ошибка отправки команды: " + command, e);
            }
        }
    }
    
    private void sendLocationToServer(Location location) {
        // Отправка координат на удаленный сервер
        new Thread(() -> {
            try {
                JSONObject data = new JSONObject();
                data.put("latitude", location.getLatitude());
                data.put("longitude", location.getLongitude());
                data.put("altitude", location.getAltitude());
                data.put("speed", location.getSpeed());
                data.put("bearing", location.getBearing());
                data.put("timestamp", System.currentTimeMillis());
                
                // HTTP POST запрос
                URL url = new URL("https://your-server.com/api/boat-location");
                HttpURLConnection conn = (HttpURLConnection) url.openConnection();
                conn.setRequestMethod("POST");
                conn.setRequestProperty("Content-Type", "application/json");
                conn.setDoOutput(true);
                
                OutputStream os = conn.getOutputStream();
                os.write(data.toString().getBytes());
                os.flush();
                os.close();
                
                int responseCode = conn.getResponseCode();
                Log.i("Server", "Координаты отправлены: " + responseCode);
                
            } catch (Exception e) {
                Log.e("Server", "Ошибка отправки координат", e);
            }
        }).start();
    }
    
    public void startAutopilot() {
        autopilotEnabled = true;
        currentWaypointIndex = 0;
        Log.i("Autopilot", "Автопилот включен");
    }
    
    public void stopAutopilot() {
        autopilotEnabled = false;
        sendMotorCommand(0); // остановка мотора
        sendRudderCommand(0); // руль прямо
        Log.i("Autopilot", "Автопилот выключен");
    }
    
    private void updateLocationDisplay(Location location) {
        runOnUiThread(() -> {
            TextView coordsText = findViewById(R.id.coordinates);
            coordsText.setText(String.format(
                "Lat: %.6f\nLon: %.6f\nSpeed: %.1f m/s\nHeading: %.1f°",
                location.getLatitude(),
                location.getLongitude(), 
                location.getSpeed(),
                location.getBearing()
            ));
        });
    }
}

// PID Controller класс
class PIDController {
    private double kp, ki, kd;
    private double integral = 0;
    private double previousError = 0;
    private double minOutput = -100;
    private double maxOutput = 100;
    
    public PIDController(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }
    
    public void setOutputLimits(double min, double max) {
        this.minOutput = min;
        this.maxOutput = max;
    }
    
    public double calculate(double error) {
        integral += error;
        double derivative = error - previousError;
        
        double output = kp * error + ki * integral + kd * derivative;
        
        // Ограничение выхода
        output = Math.max(minOutput, Math.min(maxOutput, output));
        
        previousError = error;
        return output;
    }
    
    public void reset() {
        integral = 0;
        previousError = 0;
    }
}