#pragma once

/**
 * Check whether the NxLib has at least the given version.
 */
bool checkNxLibVersion(int major, int minor)
{
	int nxLibMajor = NxLibItem()[itmVersion][itmMajor].asInt();
	int nxLibMinor = NxLibItem()[itmVersion][itmMinor].asInt();
	return (nxLibMajor > major) || (nxLibMajor == major && nxLibMinor >= minor);
}