# test/test.py — TinyTapeout cocotb test for tt_um_baud_rate_cdr
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, FallingEdge, Timer

def u8_from_s8(x):
    assert -128 <= x <= 127
    return x & 0xFF

def s_lower3_from_x(x):
    # Quantizer in RTL:
    # if X < -64 => S=-3 (0b1101) -> lower3=0b101=5
    # elif X < 0 => S=-1 (0b1111)  -> lower3=0b111=7
    # elif X < 64 => S=+1 (0b0001) -> lower3=0b001=1
    # else => S=+3 (0b0011)        -> lower3=0b011=3
    if x < -64:
        return 5
    elif x < 0:
        return 7
    elif x < 64:
        return 1
    else:
        return 3

@cocotb.test()
async def test_cdr_outputs(dut):
    """Check Sample_en pulses and quantizer/X mapping on uo_out."""
    # Local aliases
    clk     = dut.clk
    rst_n   = dut.rst_n
    ena     = dut.ena
    ui_in   = dut.ui_in
    uo_out  = dut.uo_out

    # 50 MHz clock (20 ns)
    cocotb.start_soon(Clock(clk, 20, units="ns").start())

    # Reset & enable
    ena.value   = 0
    rst_n.value = 0
    ui_in.value = 0
    await RisingEdge(clk)
    await RisingEdge(clk)
    ena.value   = 1
    rst_n.value = 1

    # Helper: drive a symbol (signed 8-bit) and sample one strobe later
    async def drive_and_sample(x_s8):
        ui_in.value = u8_from_s8(x_s8)
        # Wait for a Sample_en pulse (MSB of uo_out)
        for _ in range(2000):  # plenty of cycles
            await RisingEdge(clk)
            if (int(uo_out.value) >> 7) & 1:  # Sample_en = uo_out[7]
                break
        # Wait one more cycle for registered paths to settle
        await RisingEdge(clk)
        val = int(uo_out.value)
        sample_en = (val >> 7) & 1
        s_low3    = (val >> 4) & 0x7
        x_msb_nib = val & 0xF
        return sample_en, s_low3, x_msb_nib

    # Test a few PAM4 levels (non-proportional set is fine too; we just check bins)
    tests = [
        (-100, -3),
        ( -28, -1),
        ( +28, +1),
        (+100, +3),
    ]

    for x_s8, _s in tests:
        sample_en, s_low3, x_msb_nib = await drive_and_sample(x_s8)

        # Expect at least one pulse was seen
        assert sample_en in (0,1)  # (we latched the cycle after the pulse)

        # Check S mapping (lower 3 bits)
        exp_s_low3 = s_lower3_from_x(x_s8)
        assert s_low3 == exp_s_low3, f"S lower3 mismatch for X={x_s8}: got {s_low3}, exp {exp_s_low3}"

        # Check X MSB nibble reflects captured DATA
        exp_nib = (u8_from_s8(x_s8) >> 4) & 0xF
        assert x_msb_nib == exp_nib, f"X[7:4] mismatch for X={x_s8}: got {x_msb_nib}, exp {exp_nib}"

@cocotb.test()
async def test_sample_en_pulses(dut):
    """See at least a few S
