/**
 * Inclulde file in HTML code
 */
function include(filename) {
	return HtmlService.createHtmlOutputFromFile(filename).getContent();
}

/**
 * Get current script url
 */
function getUrl() {
	var url = ScriptApp.getService().getUrl();
	return url;
}