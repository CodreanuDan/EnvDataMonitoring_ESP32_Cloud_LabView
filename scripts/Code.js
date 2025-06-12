// App script used for receiving and sending data from GS

function doGet(e) {
  try {
    var sheet = SpreadsheetApp.getActiveSpreadsheet().getActiveSheet();

    if (e.parameter.mode === "read") {
      var data = sheet.getDataRange().getValues();

      var fromHour = parseInt(e.parameter.from) || 0;
      var toHour = parseInt(e.parameter.to) || 24;
      var filterDate = e.parameter.date; // format: "YYYY-MM-DD"

      var filteredData = [];

      for (var i = 3; i < data.length; i++) {
        var row = data[i];
        if (!row[0]) continue;

        var timestamp;

        // Verificăm și parsăm data
        if (Object.prototype.toString.call(row[0]) === "[object Date]") {
          timestamp = row[0]; // Date
        } else {
          timestamp = new Date(row[0]);
        }

        // Verificare dată validă
        if (isNaN(timestamp.getTime())) {
          Logger.log("Invalid date at row " + (i + 1) + ": " + row[0]);
          continue;
        }

        var rowDate = timestamp.toISOString().split("T")[0];
        var hour = timestamp.getHours();

        var dateMatch = !filterDate || rowDate === filterDate;
        var hourMatch = hour >= fromHour && hour < toHour;

        if (dateMatch && hourMatch) {
          filteredData.push({
            timestamp: timestamp.toISOString(),
            temperature: parseFloat(row[1]),
            humidity: parseFloat(row[2]),
            heatIndex: parseFloat(row[3]),
            adcMq2Voltage: parseFloat(row[4]),
            Ro: parseFloat(row[5]),
            iPPM_LPG: parseFloat(row[6]),
            iPPM_CO: parseFloat(row[7]),
            iPPM_Smoke: parseFloat(row[8]),
            adcRawValue: parseInt(row[9])
          });
        }
      }

      return ContentService
        .createTextOutput(JSON.stringify(filteredData))
        .setMimeType(ContentService.MimeType.JSON);
    }

    //  POST 
    var temperature = e.parameter.temperature;
    var humidity = e.parameter.humidity;
    var heatIndex = e.parameter.heatIndex;
    var adcMq2Voltage = e.parameter.adcMq2Voltage;
    var Ro = e.parameter.Ro;
    var iPPM_LPG = e.parameter.iPPM_LPG;
    var iPPM_CO = e.parameter.iPPM_CO;
    var iPPM_Smoke = e.parameter.iPPM_Smoke;
    var adcRawValue = e.parameter.adcRawValue;

    sheet.appendRow([
      new Date(),
      temperature,
      humidity,
      heatIndex,
      adcMq2Voltage,
      Ro,
      iPPM_LPG,
      iPPM_CO,
      iPPM_Smoke,
      adcRawValue
    ]);

    var response = {
      status: "Success",
      receivedData: {
        temperature,
        humidity,
        heatIndex,
        adcMq2Voltage,
        Ro,
        iPPM_LPG,
        iPPM_CO,
        iPPM_Smoke,
        adcRawValue
      }
    };

    return ContentService
      .createTextOutput(JSON.stringify(response))
      .setMimeType(ContentService.MimeType.JSON);

  } catch (error) {
    return ContentService
      .createTextOutput("Error: " + error.message);
  }
}
