/**
 * Some important links:
 * Service accounts overview
 * - https://cloud.google.com/iam/docs/service-account-overview
 * Google Sheets API Overview
 * - https://developers.google.com/sheets/api/guides/concepts
 * Best practices for using service accounts
 * - https://cloud.google.com/iam/docs/best-practices-service-accounts
 * Manage Service Accounts
 * - https://cloud.google.com/iam/docs/best-practices-service-accounts#manage-service-accounts
 * ESP32 Datalogging to Google Sheets (using Google Service Account)
 * - https://randomnerdtutorials.com/esp32-datalogging-google-sheets
 * Line Chart
 * - https://www.chartjs.org/docs/latest/charts/line.html
 * Epoch Converter Functions
 * - https://www.epochconverter.com/programming/
 * How to allow others to access a web app made from a Google Apps Script?
 * - https://stackoverflow.com/questions/17910764/how-to-allow-others-to-access-a-web-app-made-from-a-google-apps-script
 * Arduino Google Sheet Client Library for Arduino devices
 * - https://github.com/mobizt/ESP-Google-Sheet-Client
 * How to show decimals with 0 in chartJS
 * - https://stackoverflow.com/questions/75864092/how-to-show-decimals-with-0-in-chartjs
 *
 * Development URL
 * - https://script.google.com/macros/s/AKfycbz7K-RbqsqssCXGNbSldVfRJzzNM4AyqQZ-RvBWp0Y/dev?page=DataLogger
 */

/**
 * Route with load page handlers
 */
var Route = {};
Route.path = function (route, callback) {
	Route[route] = callback
}

/**
 * Google WebApp HTTP get() request handler
 *
 * Entry web page is DataLogger page, the page contains values of sensor over time.
 */
function doGet(e) {

	/// Build the routes to different web pages
	Route.path("DataLogger", loadDataLoggerPage);

	if ((!e.parameter.page) || (!Route[e.parameter.page])) {
		return loadPageNotFoundPage(e);
	}

	return Route[e.parameter.page](e)
}

/**
 * Load DataLogger page
 */
function loadDataLoggerPage(e) {
	Logger.log("Loading DataLogger page")
	Logger.log(JSON.stringify(e));
	var htmlOutput = HtmlService.createTemplateFromFile('DataLogger');
	return htmlOutput.evaluate().setSandboxMode(HtmlService.SandboxMode.IFRAME);
}

/**
 * Load PageNotFound page
 */
function loadPageNotFoundPage(e) {
	Logger.log("Loading loadPageNotFound page")
	Logger.log(JSON.stringify(e));
	var htmlOutput = HtmlService.createTemplateFromFile('PageNotFound');
	return htmlOutput.evaluate().setSandboxMode(HtmlService.SandboxMode.IFRAME);
}

///
/// COPY YOUR GOOGLE SHEET URL AND PASTE IT HERE
///
var url = "https://docs.google.com/spreadsheets/<<<<YOUR GOOGLE SHEET ID HERE>>>>";

/**
 * Grab whole data in 'Temperature' and 'Humidity' columns
 *
 * The google sheet contains three columns: Timestamp (epoch date in seconds), Temperature and Humidity
 * The rows are periodically filled by the ESP32 module.
 *
 * getRange(row, column, numRows, numColumns):
 *   Returns the range with the top left cell at the given coordinates with the given number of rows and columns.
 *
 *   var range = sheet.getRange(1, 1, 3, 3);
 *   var values = range.getValues();
 *   // Print values from a 3x3 box.
 *   for (var row in values) {
 *    for (var col in values[row]) {
 *       Logger.log(values[row][col]);
 *     }
 *   }
 */
function getSensorValuesFromGoogleSheet() {
	var ss = SpreadsheetApp.openByUrl(url);
	var ws = ss.getSheetByName("Sheet1");
	var getLastRow = ws.getLastRow();

	return ws.getRange(2, 1, getLastRow - 1, 3).getValues();
}

/**
 * Grab whole statistics (avg/max/min) data
 *
 * The google sheet contains six columns: Temperature (avgerage/max/min) and Humidity (avgerage/max/min)
 * The third row is periodically calculated every time the ESP32 pushes data.
 */
function getStatisticValuesFromGoogleSheet() {
	var ss = SpreadsheetApp.openByUrl(url);
	var ws = ss.getSheetByName("Sheet2");

	return ws.getRange(3, 1, 1, 6).getValues();
}