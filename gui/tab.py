#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2022 Bitcraze AB
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  You should have received a copy of the GNU General Public License
#  along with this program. If not, see <https://www.gnu.org/licenses/>.
#
#
#  This has been based on code developed by Bitcraze.

import logging

from PyQt5 import QtWidgets

logger = logging.getLogger(__name__)


class Tab(QtWidgets.QWidget):
    """Superclass for all tabs that implements common functions."""

    def __init__(self):
        super(Tab, self).__init__()
        self.tabName = "N/A"
        self.menuName = "N/A"
        self.enabled = True

    def getMenuName(self):
        """Return the name of the tab that will be shown in the menu"""
        return self.menuName

    def getTabName(self):
        """Return the name of the tab that will be shown in the tab"""
        return self.tabName

