import sigrokdecode as srd

class Decoder(srd.Decoder):
    api_version = 3
    id = 'opcode16'
    name = 'Microchip ICSP'
    longname = '4-bit Opcode followed by 16-bit Operand'
    desc = 'Decodes a 4-bit opcode followed by a 16-bit operand'
    license = 'gplv2+'
    inputs = ['logic']
    outputs = []
    tags = ['Embedded']

    channels = (
        {'id': 'clk', 'name': 'CLK', 'desc': 'Clock'},
        {'id': 'dat', 'name': 'DATA', 'desc': 'Data'},
    )
    
    options = (
        {'id': 'force_sr', 'desc': 'Force Sample Rate (Hz)', 'default': 0},
    )

    annotations = (
        ('bit', 'Bit'),      # 0
        ('opcode', 'Opcode'),# 1
        ('operand', 'Operand'),# 2
        ('frame', 'Frame'),   # 3
        ('timing', 'Timing'), # 4
    )

    annotation_rows = (
        ('bits', 'Bits', (0,)),
        ('timing', 'Bit Timing', (4,)),
        ('opcode', 'Opcode', (1,)),
        ('operand', 'Operand', (2,)),
        ('frame', 'Frame', (3,)),
    )
    
    binary = (
        ('bin_opcode', 'OpCode'),
        ('bin_operand', 'Operand'),
    )

    def __init__(self):
        self.reset()

    def reset(self):
        self.state = 'OPCODE'
        self.bitcount = 0
        self.opcode = 0
        self.operand = 0
        self.opcode_ss = None
        self.operand_ss = None
        self.last_end_sample = None
        self.gap_info = ""
        self.last_clk_edge = None

    def start(self):
        self.out_ann = self.register(srd.OUTPUT_ANN)
        self.out_bin_op = self.register(srd.OUTPUT_BINARY)
        self.out_bin_dat = self.register(srd.OUTPUT_BINARY)
        self.put(0, 0, self.out_ann, [4, ['']])
        
    def decode(self):
        while True:
            pins = self.wait({0: 'f'})
            if not pins:
                break
                
            curr_sample = self.samplenum
            
            # Get the ratio (e.g., 10.0 if sampling at 10MHz)
            spu = self.options.get('samples_per_us', 1.0)
            if spu <= 0: spu = 1.0 # Prevent division by zero

            if self.last_clk_edge is not None:
                delta_samples = curr_sample - self.last_clk_edge
                
                # Direct calculation to microseconds
                period_us = delta_samples / spu
                
                # Logic to show 'us' for short times and 'ms' for long gaps
                if period_us >= 1000:
                    timing_label = f"{period_us/1000:.2f}ms"
                else:
                    timing_label = f"{period_us:.1f}us"
                
                self.put(self.last_clk_edge, curr_sample, self.out_ann, [4, [timing_label]])
            
            self.last_clk_edge = curr_sample
            self._handle_bit(pins[1], curr_sample)

    def _handle_bit(self, bit, sample):
        # 1. Annotate individual bits
        self.put(sample - 5, sample, self.out_ann, [0, [str(bit)]])

        # SAFE ACCESS for gap calculation
        sr = getattr(self, 'samplerate', None)

        if self.state == 'OPCODE':
            if self.bitcount == 0:
                self.opcode_ss = sample
                self.opcode = 0
                
                if self.last_end_sample is not None and sr and sr > 0:
                    gap_samples = sample - self.last_end_sample
                    gap_ms = (gap_samples / sr) * 1000
                    if gap_ms < 1.0:
                        self.gap_info = f", Gap:{gap_ms * 1000:.1f}us"
                    else:
                        self.gap_info = f", Gap:{gap_ms:.3f}ms"
                else:
                    self.gap_info = ""

            self.opcode |= (bit << self.bitcount)
            self.bitcount += 1

            if self.bitcount == 4:
                self.put(self.opcode_ss, sample, self.out_ann, [1, [f'OP=0x{self.opcode:X}']])
                self.state = 'OPERAND'
                self.bitcount = 0

        else:  # OPERAND state
            if self.bitcount == 0:
                self.operand_ss = sample
                self.operand = 0

            self.operand |= (bit << self.bitcount)
            self.bitcount += 1

            if self.bitcount == 16:
                self.last_end_sample = sample
                self.put(self.operand_ss, sample, self.out_ann, [2, [f'DATA=0x{self.operand:04X}']])
                
                full_text = f"OP:0x{self.opcode:X}, DATA:0x{self.operand:04X}{self.gap_info}"
                self.put(self.opcode_ss, sample, self.out_ann, [3, [full_text]])
                
                # Reset
                self.state = 'OPCODE'
                self.bitcount = 0