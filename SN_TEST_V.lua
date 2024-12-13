-- Define a function to convert unsigned 16-bit to signed 16-bit (uint16_t [0 - 65635], int16_t [-32768 - 32767])
function unsignedToSigned16bit(value)
    if value >= 32768 then
        return value - 65536
    else
        return value
    end
end

-- Function to convert a 16-bit value to a percentage with two decimal places
function convert_to_percentage(value)    
    -- Calculate the percentage
    local percentage = (value / 65535) * 100
    
    -- Format the percentage to two decimal places
    return string.format("%.2f", percentage)
end

-- Define a function to parse payload and decode sensor data
function parsePayload(appeui, deveui, payloadIn)
    -- Decode the payload into individual variables
    payload, Error = resiot_hexdecode(payloadIn)
  	if Error ~= "" then
    	-- error 
        --resiot_debug(Error)
    else
        -- value read correctly
        --resiot_debug(ArrByte)
    end
  	-- ACCELEROMETER --
  	local ax_array = {payload[1], payload[2]}
  	local ax_16bit = resiot_ba2intLE16(ax_array)
    ax_16bit = unsignedToSigned16bit(ax_16bit) -- Convert to signed
    local ax = (ax_16bit / 4095) * 9.81
  
    local ay_array = {payload[3], payload[4]}
  	local ay_16bit = resiot_ba2intLE16(ay_array)
    ay_16bit = unsignedToSigned16bit(ay_16bit)
  	local ay = (ay_16bit / 4095) * 9.81
  
    local az_array = {payload[5], payload[6]}
  	local az_16bit = resiot_ba2intLE16(az_array)
  	az_16bit = unsignedToSigned16bit(az_16bit)
  	local az = (az_16bit / 4095) * 9.81

  	-- Si7021 --
    local temperature_array = {payload[7], payload[8]}
  	local temperature_16bit = resiot_ba2intLE16(temperature_array)
  	local temperature = ((175.72 * temperature_16bit) / 65536) - 46.85
  
    local humidity_array = {payload[9], payload[10]}
 	local humidity_16bit = resiot_ba2intLE16(humidity_array)
  	local humidity = ((125 * humidity_16bit) / 65536) - 6
  
  	-- RAW ANALOGIC SENSORS --
    local moisture_array = {payload[11], payload[12]}
 	local moisture_16bit = resiot_ba2intLE16(moisture_array)
  	local moisture = convert_to_percentage(moisture_16bit)
  
    local light_array = {payload[13], payload[14]}
 	local light_16bit = resiot_ba2intLE16(light_array)
  	local light = convert_to_percentage(light_16bit)
  
	-- TCS34725 --
  	local clear_array = {payload[15], payload[16]}
 	local clear = resiot_ba2intLE16(clear_array)
  
  	local red_array = {payload[17], payload[18]}
 	local red = resiot_ba2intLE16(red_array)
  
  	local green_array = {payload[19], payload[20]}
 	local green = resiot_ba2intLE16(green_array)
  
  	local blue_array = {payload[21], payload[22]}
 	local blue = resiot_ba2intLE16(blue_array)
  
	-- GPS --
  	local Latitude_array = {payload[23], payload[24], payload[25], payload[26]}
 	local Latitude = resiot_ba2float32LE(Latitude_array)
  
  	local Longitude_array = {payload[27], payload[28], payload[29], payload[30]}
 	local Longitude = resiot_ba2float32LE(Longitude_array)

  	-- Log payload bytes
    --for i = 1, #payload do
        -- Print each byte in decimal and hexadecimal formats
        --resiot_debug(string.format("Hex %d = 0x%02X", i, payload[i]))
    --end
  
    -- Log decoded values
    --resiot_debug(string.format("Decoded Values: Ax=%d, Ay=%d, Az=%d, Temp=%d, Humidity=%d, Moist=%d, Light=%d, C=%d, R=%d, G=%d, B=%d, Lat=%.6f, Lon=%.6f", ax_16bit, ay_16bit, az_16bit, temperature_16bit, humidity_16bit, moisture_16bit, light_16bit, clear, red, green, blue, Latitude, Longitude))

      -- Log processed values
    --resiot_debug(string.format("Processed Values: Ax=%.2f, Ay=%.2f, Az=%.2f, Temp=%.2f, Humidity=%.2f, Moist=%d, Light=%d, C=%d, R=%d, G=%d, B=%d, Lat=%.6f, Lon=%.6f", ax, ay, az, temperature, humidity, moisture, light, clear, red, green, blue, Latitude, Longitude))
  
    -- Set values to ResIOT tags
    local worked, err

  -- ---------------------------------------------------------------------------------------
  
    worked, err = resiot_setnodevalue(appeui, deveui, "ax", ax)
    if not worked then
        --resiot_debug(string.format("Error setting ax: %s", err))
    end
  
    worked, err = resiot_setnodevalue(appeui, deveui, "ay", ay)
    if not worked then
        --resiot_debug(string.format("Error setting ay: %s", err))
    end
  
    worked, err = resiot_setnodevalue(appeui, deveui, "az", az)
    if not worked then
        --resiot_debug(string.format("Error setting az: %s", err))
    end

-- ---------------------------------------------------------------------------------------
  
    worked, err = resiot_setnodevalue(appeui, deveui, "temperature", temperature)
    if not worked then
        --resiot_debug(string.format("Error setting temperature: %s", err))
    end

    worked, err = resiot_setnodevalue(appeui, deveui, "humidity", humidity)
    if not worked then
        --resiot_debug(string.format("Error setting humidity: %s", err))
    end
  
-- ---------------------------------------------------------------------------------------
  
    worked, err = resiot_setnodevalue(appeui, deveui, "moisture", moisture)
    if not worked then
        --resiot_debug(string.format("Error setting moisture: %s", err))
    end
  
    worked, err = resiot_setnodevalue(appeui, deveui, "light", light)
    if not worked then
        --resiot_debug(string.format("Error setting light: %s", err))
    end
  
-- ---------------------------------------------------------------------------------------
 
    worked, err = resiot_setnodevalue(appeui, deveui, "clear", clear)
    if not worked then
        --resiot_debug(string.format("Error setting clear: %s", err))
    end
  
    worked, err = resiot_setnodevalue(appeui, deveui, "red", red)
    if not worked then
        --resiot_debug(string.format("Error setting red: %s", err))
    end

    worked, err = resiot_setnodevalue(appeui, deveui, "green", green)
    if not worked then
        --resiot_debug(string.format("Error setting green: %s", err))
    end

    worked, err = resiot_setnodevalue(appeui, deveui, "blue", blue)
    if not worked then
        --resiot_debug(string.format("Error setting blue: %s", err))
    end
  
-- ---------------------------------------------------------------------------------------

    worked, err = resiot_setnodevalue(appeui, deveui, "Latitude", Latitude)
    if not worked then
        --resiot_debug(string.format("Error setting Latitude: %s", err))
    end

    worked, err = resiot_setnodevalue(appeui, deveui, "Longitude", Longitude)
    if not worked then
        --resiot_debug(string.format("Error setting Longitude: %s", err))
    end
  

    --resiot_debug("All values successfully processed.")
end

-- Main script execution
Origin = resiot_startfrom() -- Scene process starts here

if Origin == "Manual" then -- Manual script execution for testing
    -- Generate a random payload
    math.randomseed(os.time())
    payload = ""
    for i = 1, 30 do
        payload = payload .. string.format("%02X", math.random(0, 255))
    end
  
    appeui = "70b3d57ed000ac4a"
    deveui = "8639323559379194"
else -- Normal execution, get payload received from device
    appeui = resiot_comm_getparam("appeui")
    deveui = resiot_comm_getparam("deveui")
    payload, err = resiot_getlastpayload(appeui, deveui)
    if not payload then
        --resiot_debug(string.format("Error getting payload: %s", err))
        return
    end
end

-- Process the payload
parsePayload(appeui, deveui, payload)
