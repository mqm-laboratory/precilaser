import logging
from typing import Optional, Tuple

from .device import AbstractPrecilaserDevice
from .enums import PrecilaserCommand, PrecilaserDeviceType
from .status import SeedStatus

logger = logging.getLogger(__name__)


class Seed(AbstractPrecilaserDevice):
    def __init__(
        self,
        resource_name: str,
        address: int,
        header: bytes = b"P",
        terminator: bytes = b"\r\n",
        device_type: PrecilaserDeviceType = PrecilaserDeviceType.SEED,
        endian: str = "big",
    ):
        super().__init__(
            resource_name, address, header, terminator, device_type, endian
        )
        self.serial: Optional[bytes] = None
        self.wavelength_params: Optional[Tuple[int, ...]] = None

    def __exit__(self):
        self.rm.close()

    def _set_value(
        self,
        value: int,
        command: PrecilaserCommand,
        nbytes: int = 2,
        save: bool = False,
    ):
        payload = value.to_bytes(nbytes, self.endian)
        if save:
            payload += b"1"
        else:
            payload += b"0"
        message = self._generate_message(command, payload)
        self._write(message)
        return

    @property
    def status(self) -> SeedStatus:
        message = self._generate_message(PrecilaserCommand.SEED_STATUS)
        self._write(message)
        message = self._read()

        if message is None:
            logger.warning("No response from device when getting status")
            return

        if message.payload is not None:
            return SeedStatus(message.payload, self.endian)
        else:
            raise ValueError("no status data bytes retrieved")

    @property
    def temperature_setpoint(self) -> float:
        return self.status.temperature_set

    @temperature_setpoint.setter
    def temperature_setpoint(self, temperature: float):
        setpoint = int(temperature * 1_000)
        self._set_value(setpoint, PrecilaserCommand.SEED_SET_TEMP)
        message = self._read()

        if message is None:
            logger.warning("No response from device when setting temperature setpoint")
            return

        if message.payload is not None:
            self._check_write_return(
                message.payload[:2], setpoint, "temperature setpoint"
            )
        else:
            raise ValueError(f"not set to requested value: {setpoint}")

    @property
    def piezo_voltage(self) -> float:
        return self.status.piezo_voltage

    @piezo_voltage.setter
    def piezo_voltage(self, voltage: float):
        assert voltage >= 0 and voltage <= 74, (
            "Piezo voltage cannot exceed 0V-74V range"
        )
        setpoint = int(voltage * 100)
        self._set_value(setpoint, PrecilaserCommand.SEED_SET_VOLTAGE)

        message = None
        for _ in range(2):
            message = self._read()
            if message is not None:
                break

        if message is None:
            logger.warning("No response from device when setting piezo voltage")
            return

        if message.payload is not None:
            self._check_write_return(message.payload[:2], setpoint, "piezo voltage")
        else:
            raise ValueError(f"not set to requested value: {setpoint}")

    # def enable(self):
    #     # supplied programming manual is incorrect, e.g. also the command and return
    #     # enums
    #     # self._set_value(True, PrecilaserCommand.SEED_ENABLE, nbytes=1)
    #     # byte 13 is set to 1
    #     enable_bytes = (
    #         b"P\x00d\xa7\x19a\xa8a\xa8\x01\xf4\x00\x00\x00\x00\x00\x00\x00\x01\x01d"
    #         b"\x00\x00\x00\x00\x00\x00\x00\x00\x00\x91K\r\n"
    #     )
    #     self.instrument.write_raw(enable_bytes)
    #     # enabling/disabling the laser returns a status message, where in the first
    #     # received message after sending the command bit 13 is set to indicate
    #     # a change in emission state
    #     message = self._read()
    #     if message.payload is None or message.payload[13] != 1:
    #         raise ValueError("emission not enabled")
    #     # message = self._read()
    #     # self._check_write_return(message.payload, True, "enable laser")

    # def disable(self):
    #     # supplied programming manual is incorrect, e.g. also the command and return
    #     # enums
    #     # self._set_value(False, PrecilaserCommand.SEED_ENABLE, nbytes=1)
    #     # byte 13 is set to 0
    #     disable_bytes = (
    #         b"P\x00d\xa7\x19a\xa8a\xa8\x01\xf4\x00\x00\x00\x00\x00\x00\x00\x00\x01d\x00"
    #         b"\x00\x00\x00\x00\x00\x00\x00\x00\x90J\r\n"
    #     )
    #     self.instrument.write_raw(disable_bytes)
    #     # enabling/disabling the laser returns a status message, where in the first
    #     # received message after sending the command bit 13 is set to indicate
    #     # a change in emission state
    #     message = self._read()
    #     if message.payload is None or message.payload[13] != 1:
    #         raise ValueError("emission not disabled")
    #     # message = self._read()
    #     # self._check_write_return(message.payload, False, "disable laser")

    def _get_serial_wavelength_params(self):
        message = self._generate_message(PrecilaserCommand.SEED_SERIAL_WAV)
        self._write(message)
        message = self._read()

        if message is None:
            logger.warning(
                "No response from device when getting serial wavelength params"
            )
            return

        self.serial = message.payload[16:24]
        parameter_bytes = message.payload[25 : 25 + 64]
        self.wavelength_params = [parameter_bytes[i] for i in range(6)]

    @property
    def wavelength(self) -> float:
        status = self.status
        temp_grating_act = status.temperature_act * 1_000
        assert self.wavelength_params is not None, (
            "Wavelength parameters not loaded from device"
        )
        parameter = self.wavelength_params
        wavelength = (
            (parameter[0] << 8) | parameter[1]
        ) * temp_grating_act / 10_000 + (
            parameter[2] << 24
            | parameter[3] << 16
            | ((parameter[4] << 8) | parameter[5])
        )
        # manual states / 1_000 but this yields an incorrect wavelength
        return wavelength / 10_000

    @wavelength.setter
    def wavelength(self, wavelength: float):
        status = self.status
        assert self.wavelength_params is not None, (
            "Wavelength parameters not loaded from device"
        )
        parameter = self.wavelength_params
        temp_grating_act = (
            (
                wavelength * 10_000
                - (
                    parameter[2] << 24
                    | parameter[3] << 16
                    | ((parameter[4] << 8) | parameter[5])
                )
            )
            * 10_000
            / ((parameter[0] << 8) | parameter[1])
        )
        temp_set = temp_grating_act / 1_000
        self.temperature_setpoint = temp_set
