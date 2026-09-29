#!/usr/bin/env python3
"""
Unit and integration tests for web static assets and HTTP teleop dashboard server
"""

import sys
import os
import re
import unittest
import urllib.request
import urllib.error

SERVER_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if SERVER_DIR not in sys.path:
    sys.path.insert(0, SERVER_DIR)

from pilot_server import PilotServerManager

class TestWebAssetsAndHttp(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.web_dir = os.path.join(SERVER_DIR, "pilot_web")
        cls.index_path = os.path.join(cls.web_dir, "index.html")
        cls.css_path = os.path.join(cls.web_dir, "style.css")
        cls.js_path = os.path.join(cls.web_dir, "app.js")

    def test_static_files_exist_and_non_empty(self):
        self.assertTrue(os.path.isfile(self.index_path), "index.html must exist")
        self.assertTrue(os.path.isfile(self.css_path), "style.css must exist")
        self.assertTrue(os.path.isfile(self.js_path), "app.js must exist")

        self.assertGreater(os.path.getsize(self.index_path), 500)
        self.assertGreater(os.path.getsize(self.css_path), 500)
        self.assertGreater(os.path.getsize(self.js_path), 500)

    def test_dom_ids_contract_between_html_and_js(self):
        """Verify all document.getElementById calls in app.js exist in index.html"""
        with open(self.js_path, "r", encoding="utf-8") as f:
            js_code = f.read()

        with open(self.index_path, "r", encoding="utf-8") as f:
            html_code = f.read()

        # Find all getElementById occurrences in JS
        js_ids = set(re.findall(r"document\.getElementById\(['\"]([a-zA-Z0-9_-]+)['\"]\)", js_code))
        self.assertGreater(len(js_ids), 10, "Should have found getElementById calls")

        # Find all id="..." occurrences in HTML
        html_ids = set(re.findall(r'id=["\']([a-zA-Z0-9_-]+)["\']', html_code))

        # Check for missing element IDs
        missing_ids = js_ids - html_ids
        self.assertEqual(len(missing_ids), 0, f"HTML is missing DOM elements referenced in JS: {missing_ids}")

    def test_http_server_serving(self):
        """Test HTTP server serves index, css, js, and handles 404"""
        manager = PilotServerManager(
            web_dir=self.web_dir,
            http_port=0,  # Ephemeral port
            ws_port=0,
            chica_host="127.0.0.1",
            chica_port=18711,
            mock=True
        )
        manager.start_http_server()
        port = manager.http_port
        base_url = f"http://127.0.0.1:{port}"

        try:
            # 1. Fetch index.html
            with urllib.request.urlopen(f"{base_url}/") as resp:
                self.assertEqual(resp.status, 200)
                body = resp.read().decode("utf-8")
                self.assertIn("MakeYourPet Pilot HUD", body)

            # 2. Fetch style.css
            with urllib.request.urlopen(f"{base_url}/style.css") as resp:
                self.assertEqual(resp.status, 200)
                body = resp.read().decode("utf-8")
                self.assertIn(".pilot-container", body)

            # 3. Fetch app.js
            with urllib.request.urlopen(f"{base_url}/app.js") as resp:
                self.assertEqual(resp.status, 200)
                body = resp.read().decode("utf-8")
                self.assertIn("VirtualJoystick", body)

            # 4. Fetch 404 nonexistent file
            with self.assertRaises(urllib.error.HTTPError) as ctx:
                urllib.request.urlopen(f"{base_url}/nonexistent_file_404.html")
            self.assertEqual(ctx.exception.code, 404)

        finally:
            manager.stop()


if __name__ == "__main__":
    unittest.main()
