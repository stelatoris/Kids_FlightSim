#pragma once

//---------------------------------

double floatMap(double x, double in_min, double in_max, double out_min, double out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

unsigned long startTime;
enum APState
{
    STATE_IDLE,
    STATE_SET
} state = STATE_IDLE;

struct Fuel_tank
{

    Fuel_tank(double cap, double quant) : cpcty{cap}, qty{quant} {}
    Fuel_tank() : cpcty{10000}, qty{1000} {}

    double get_quantity() const { return qty; }
    void set_quantity(double q) { qty = q; }
    void set_refuel(int q);
    void consume(double q) { qty -= q; }
    void set_capacity(double c) { cpcty = c; }
    double get_capacity() const { return cpcty; }

    void refuel();

private:
    double cpcty;
    double qty;
    int refuel_qty;
};

//---------------------------------

struct Engine
{
    Engine(Fuel_tank &f, int start_pin) : tank{f}, eng_start_pin{start_pin} { thrtl_flip = false; }

    bool start_btn_state() { return digitalRead(eng_start_pin); }
    bool start_up = true;
    double rpm();
    double get_rpm() { return eng_rpm; }
    double get_temp() { return eng_temp; }

    float get_thrust();
    double fuel_flow();
    double get_throttle();

    void calculateEngineTemperature(float fuel_flow);
    void calculateEngineTemperature(float fuelFlowRate, float rpm);
    void EngTemp(double fuelFlowRate, double airspeed);
    bool engineON() { return eng_ON; }
    void setON() { eng_ON = true; }
    void set_throttle(double t) { throttle = t; }
    void flip_throttle(bool b) { thrtl_flip = b; } // flips rotation on potentiometer
    bool get_throttle_axis() const { return thrtl_flip; }
    void set_F_cut_off(int s) { fuel_cut_off = s; }

    void get_readings();
    // int eng_num;

    int EngStatus_LED;
    int Eng_ABurner_LED;
    int Eng_Hot_LED;
    int Eng_Fire_LED;
    double eng_temp = 0.0;
    bool eng_ON;
    bool f_pump_ON;

private:
    Fuel_tank &tank;
    int eng_start_pin;
    double throttle;
    double eng_rpm;

    // void set_temperature();
    float thrust = 0.0f;
    double gps = 1.0; // gives gallons per second at 100% throttle
    double f_flow;

    bool thrtl_flip;
    bool fuel_cut_off;
    bool after_burner = false;
    bool engine_fire = false;
};

void print_stats()
{
    /*
    ///Serial.print("\t Time(millis): ");
    //Serial.print(millis());
    //Serial.print("\t Time(s): ");
    //Serial.print(seconds);
    Serial.print("\t Fuel Amount: ");
    Serial.print(tank.get_quantity());
    Serial.print("\t Fuel flow: ");
    Serial.print(engine1.fuel_flow());
  Serial.print("\t Fuel Low: ");
    Serial.print(digitalRead(fuel_L_LED));

    Serial.print("\t Throttle 1: ");
    Serial.print(engine1.get_throttle());
    Serial.print("\t Throttle 2: ");
    Serial.print(engine2.get_throttle());
    Serial.print("\t RPM 1: ");
    Serial.print(engine1.rpm());
    Serial.print("\t RPM 2: ");
    Serial.print(engine2.rpm());
    Serial.print("\t E1 On: ");
    Serial.print(engine1.engineON());
    Serial.print("\t E2 On: ");
    Serial.print(engine2.engineON());
    //Serial.print("\t Speed: ");
    //Serial.print(speed_v);
    //Serial.print("\t F_pump: ");
    //Serial.print(f_pump_sw_state);
    Serial.println();
    */
    /*
    Serial.print("\t Speed: ");
    Serial.print(speed_v);


    Serial.print("\t drag: ");
    Serial.print(airplane.total_fC());
    Serial.print("\t name: ");
    Serial.print(airplane.drag[0].f_name);
    Serial.print("\t cD: ");
    Serial.print(airplane.drag[0].cD);
    Serial.print("\t Status: ");
    Serial.print(airplane.drag[0].f_status);

        Serial.print("\t name: ");
    Serial.print(airplane.drag[1].f_name);
    Serial.print("\t cD: ");
    Serial.print(airplane.drag[1].cD);
    Serial.print("\t Status: ");
    Serial.print(airplane.drag[1].f_status);
    Serial.println();
    */

    // Serial.println();
}

double airDensity = 1.225;       // kg/m^3 (density of air at sea level and STP)
double engineAirflowArea = 3.14; // m^2 (example value for a turbofan engine with 2 m fan diameter)
// float airspeed = 250.0; // m/s (example value for a commercial airliner at cruise speed)

// Set the initial values
double fuelFlowRate = 0.1;              // kg/s
double airFlowRate = 1.0;               // kg/s
double temperature = 300.0;             // K
double heatTransferRate = 10000.0;      // J/s
double heatCapacity = 300.0;            // J/K
double ambientTemperature = 293.0;      // K
double heatTransferCoefficient = 100.0; // W/m2K
double surfaceArea = 1.0;               // m2
double afterBurnerFactor = 1.2;

// Set the simulation time step
double timeStep = 1.0 / 60.0;

void Engine::EngTemp(double fuelFlowRate, double airspeed)
{
    double temperatureDifference = temperature - ambientTemperature;
    // Check if the temperature difference is zero
    if (fabs(temperatureDifference) < 0.0001)
    {
        temperatureDifference = 0.0001;
    }
    // Serial.print("\ttempDiff: ");
    // Serial.print(temperatureDifference);
    double heatTransferRateToEnvironment = heatTransferCoefficient * surfaceArea * temperatureDifference / 1000;
    // Serial.print("\theatTransfer: ");
    // Serial.print(heatTransferRateToEnvironment);
    airFlowRate = airDensity * engineAirflowArea * airspeed;
    // Serial.print("\tairFlowe: ");
    // Serial.print(airFlowRate);
    //  Calculate the temperature change per second
    double temperatureChange{0};
    if (after_burner)
    {
        temperatureChange = ((fuelFlowRate * heatTransferRate * afterBurnerFactor) / 320 - heatTransferRateToEnvironment / 10 - airFlowRate / 200) / 100;
    }
    else
        temperatureChange = ((fuelFlowRate * heatTransferRate) / 320 - heatTransferRateToEnvironment / 10 - airFlowRate / 200) / 100;
    // Serial.print("\ttemperatureChange: ");
    // Serial.print(temperatureChange);
    //  Update the temperature
    temperature += temperatureChange;
    if (fabs(temperature - ambientTemperature) < 0.1)
    {
        temperature = ambientTemperature;
    }

    // Set the temperature of the engine
    eng_temp = temperature;
    // Serial.print("\teng_temp: ");
    // Serial.println(eng_temp);
}