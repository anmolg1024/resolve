library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;


entity pipeline_datapath is
  port (
	clk    : in std_logic; --from outside
	reset  : in std_logic --clr for flushing etc. - clear is selective
  ) ;
end entity ; -- pipeline_datapath

architecture arch of pipeline_datapath is

component LM_SM_Block is

	port
	(
		clk		 			: in	std_logic;
		LMSM					: in	std_logic_vector(1 downto 0);
		reset	 				: in	std_logic;
		valid					: in 	std_logic;
		mux_rr_adw_control		: out	std_logic;
		mux_rr_addr2_rf_control : out	std_logic;
		mux_rr_dra_control		: out	std_logic;
		enable					: out	std_logic

	);

	
end component;

------------------------------------------------------

component id_adw_control is
  port (
	op	  : in std_logic_vector(3 downto 0);
	id_adw_mux_control : out std_logic_vector(1 downto 0) 
  ) ;
end component ; -- id_adw_control

-----------------------------------------------------


component adder is
   generic(input_width : integer := 16);
   port(reg_a, reg_b: in std_logic_vector(input_width-1 downto 0);
        add_out: out std_logic_vector(input_width-1 downto 0)
        );
end component;

component ALU is
   port(reg_a, reg_b: in std_logic_vector(15 downto 0);
        OP: in std_logic_vector(3 downto 0);
        ALU_out: out std_logic_vector(15 downto 0);
        carry: out std_logic;
        zero: out std_logic;
		    cmp: out std_logic
		  );
end component;

component increment is

 port(
 	din 	: in STD_LOGIC_VECTOR(15 downto 0); 
 	dout	: out STD_LOGIC_VECTOR(15 downto 0) 
 	);
end component;

component instr_decoder is  
  port(clock,reset,clr : in std_logic;
       din   		 : in std_logic_vector(15 downto 0);  
       dout  	    : out std_logic_vector(15 downto 0)
      );  
end component;


component lm_sm_decoder is
  port (
  clk		:in std_logic;
	din    		: in  STD_LOGIC_VECTOR(7 downto 0);
	op			: in  STD_LOGIC_VECTOR(3 downto 0);
	LM_SM       : out STD_LOGIC_VECTOR(1 downto 0);
    dout   		: out STD_LOGIC_VECTOR(7 downto 0);
    Reg_Address : out STD_LOGIC_VECTOR(2 downto 0);
	valid  		: out STD_LOGIC :='1';
	control_bit : out STD_LOGIC := '1'

  ) ;
end component ; -- lm_sm_decoder updated 

component MEM IS
	GENERIC
	(
		ADDRESS_WIDTH	: integer := 16;
		DATA_WIDTH	: integer := 16
	);
	PORT(
		clock				: IN  std_logic;
		data_in				: IN  std_logic_vector(	DATA_WIDTH - 1 DOWNTO 0);
		address				: IN  std_logic_vector(ADDRESS_WIDTH - 1 DOWNTO 0);
		wr_en				: IN  std_logic;
		data_out			: OUT std_logic_vector(DATA_WIDTH - 1 DOWNTO 0)
	);
END component;

component mux2to1 is
generic(input_width : integer);
port(
	d0 : STD_LOGIC_VECTOR(input_width-1 downto 0);
	d1 : STD_LOGIC_VECTOR(input_width-1 downto 0);

  	sel: in STD_LOGIC;
	dout: out STD_LOGIC_VECTOR(input_width-1 downto 0)
);
end component;

component mux4to1 is
generic(input_width : integer);

port(
	d0 : STD_LOGIC_VECTOR(input_width-1 downto 0);
	d1 : STD_LOGIC_VECTOR(input_width-1 downto 0);
  d2 : STD_LOGIC_VECTOR(input_width-1 downto 0);
  d3 : STD_LOGIC_VECTOR(input_width-1 downto 0);

  sel: in STD_LOGIC_VECTOR(1 downto 0);
	dout: out STD_LOGIC_VECTOR(input_width-1 downto 0)
);
end component;

component my_reg is
	generic ( data_width : integer);
	port(
		clk, en, clr,reset: in std_logic;
		din: in std_logic_vector(data_width-1 downto 0);
		dout: out std_logic_vector(data_width-1 downto 0));
end component;

component my_reg_one is
	port(
		clk, en, clr, reset: in std_logic;
		din: in std_logic;
		dout: out std_logic
		);
end component;

component regfile is   
    port
    (
    rf_d1          	: out std_logic_vector(15 downto 0);
    rf_d2          	: out std_logic_vector(15 downto 0);
    rf_d3          	: in  std_logic_vector(15 downto 0);
	R7_in			: in  std_logic_vector(15 downto 0);

	R7_enable		: in  std_logic;
    wr_enable     	: in  std_logic;

    Addr1   	    : in  std_logic_vector(2 downto 0);
    Addr2	        : in  std_logic_vector(2 downto 0);
    Addr3   	    : in  std_logic_vector(2 downto 0);

    clk,reset 	        : in  std_logic
    );
end component;

component IM IS

	PORT(
		address				: IN  std_logic_vector(15 DOWNTO 0);
		data_out			: OUT std_logic_vector(15 DOWNTO 0)
	);
END component;


component sign_ext is

 port(
 	imm6 	: in STD_LOGIC_VECTOR(5 downto 0); --lw,sw,beq,adi
 	imm8	: in STD_LOGIC_VECTOR(7 downto 0); --lm,sm
 	imm9	: in STD_LOGIC_VECTOR(8 downto 0); --jal,lhi

 	op_code : in STD_LOGIC_VECTOR(3 downto 0);
 	dout	: out STD_LOGIC_VECTOR(15 downto 0) 
 	);
end component;

component TwosComplement is
port (
  input: in std_logic_vector(15 downto 0);
  output: out std_logic_vector(15 downto 0)
);

end component;
-----------------------------------------------------------------------------
component controlblock_MEM is
  port (
	valid 		: in std_logic ;
	flags_WB    : in std_logic_vector(1 downto 0);
	op			: in std_logic_vector(3 downto 0);
	op_type		: in std_logic_vector(1 downto 0) ;
	Mux_MEM_ADDR_Control: out std_logic ;
	Mux_MEM_DIN_Control : out std_logic ;
	Mux_MEM_PC_Control : out std_logic ;
	Mem_wr_en			: out std_logic; 
	adw					: in std_logic_vector(2 downto 0);
	LMSM 					: in std_logic_vector(1 downto 0)


  ) ;
end component;

component controlblock_EX is
	port(
		op: in std_logic_vector(3 downto 0);
		valid: in std_logic;
		cmp: in std_logic;
		mux_ex_pc_imm_control:out std_logic_vector(1 downto 0);
		mux_ex_alu1_control:out std_logic;
		mux_ex_alu2_control:out std_logic_vector(1 downto 0));
end component;

component wb_control is
	port(
		reg_flag_wb_rf: in std_logic_vector(1 downto 0);
		op_type: in std_logic_vector (1 downto 0);
		op: in std_logic_vector(3 downto 0);
		mux_wb_pc_control: out std_logic;
		mux_wb_rf_d3_control: out std_logic_vector(1 downto 0);
		wr_en: out std_logic;
		adw: in std_logic_vector(2 downto 0);
		mux_r7_in_control : out std_logic;
		valid		: in std_logic ;
		LMSM 		: in std_logic_vector(1 downto 0)		
	);
end component;
 
 component rr_mux_control is
	port(
	op: in std_logic_vector(3 downto 0);
	ara:in std_logic_vector(2 downto 0);
	mux_rr_control:out std_logic
	);
end component;

------------------------------------------------------------------------


component IF_ID is
	port(     
		pc_in 		: in std_logic_vector(15 downto 0);
		ir_in 		: in std_logic_vector(15 downto 0);
		pc_inc_in	: in std_logic_vector(15 downto 0);
		clk			: in std_logic;
		clr			: in std_logic;
		reset       : in std_logic;
		en 			: in std_logic;
		-------------------------------------------
		pc_out 		: out std_logic_vector(15 downto 0);
		ir_out		: out std_logic_vector(15 downto 0);
		pc_inc_out	:out std_logic_vector(15 downto 0)
		);

end component;

component ID_RR is  
	port(
		clk			: in std_logic;
		clr 		: in std_logic;
		reset       : in std_logic;
		en 			: in std_logic;

		pc_imm_in	: in std_logic_vector(15 downto 0);
		imm_in  	: in std_logic_vector(15 downto 0);

		ara_in		: in std_logic_vector(2 downto 0);
		arb_in		: in std_logic_vector(2 downto 0);
		adw_in		: in std_logic_vector(2 downto 0);
		

		pc_inc_in 	: in std_logic_vector(15 downto 0);
		type_in		: in std_logic_vector(1 downto 0);

		valid_in    : in  std_logic;
		valid_out	: out std_logic;

		--------------------------------------------

		pc_imm_out 	: out std_logic_vector(15 downto 0);
		imm_out   	: out std_logic_vector(15 downto 0);

		ara_out		: out std_logic_vector(2 downto 0);
		arb_out		: out std_logic_vector(2 downto 0);
		adw_out		: out std_logic_vector(2 downto 0);
		

		pc_inc_out 	: out std_logic_vector(15 downto 0);
		type_out	: out std_logic_vector(1 downto 0);
	-------------------------------------------
		op_in		: in std_logic_vector(3 downto 0);
		op_out		: out std_logic_vector(3 downto 0));


end component;

component RR_EX is
	port (
		clk			: in std_logic;
		clear		: in std_logic;
		reset       : in std_logic;
		en 			: in std_logic;
		----------------------------------------------

		PC_Imm_in 	: in std_logic_vector(15 downto 0);
		Imm_in 		: in std_logic_vector(15 downto 0);
		OP_in 		: in std_logic_vector(3 downto 0);
		DRA_in 		: in std_logic_vector(15 downto 0);
		DRB_in 		: in std_logic_vector(15 downto 0);
		ADW_in 		: in std_logic_vector(2 downto 0);
		Type_in 	: in std_logic_vector(1 downto 0);
	
		PC_Inc_in 	: in std_logic_vector(15 downto 0);
		---------------------------------------------
		PC_Imm_out	: out std_logic_vector(15 downto 0);
		Imm_out 	: out std_logic_vector(15 downto 0);
		OP_out 		: out std_logic_vector(3 downto 0);
		DRA_out 	: out std_logic_vector(15 downto 0);
		DRB_out 	: out std_logic_vector(15 downto 0);
		ADW_out 	: out std_logic_vector(2 downto 0);
		Type_out 	: out std_logic_vector(1 downto 0);

		PC_Inc_out 	: out std_logic_vector(15 downto 0);

		valid_in    : in  std_logic;
		valid_out	: out std_logic;
		LMSM_in			: in std_logic_vector( 1 downto 0);
		LMSM_out			: out std_logic_vector( 1 downto 0)
		);

end component;

component EX_MEM is
	port (
		clk			: in std_logic;
		clear		: in std_logic;
		reset       : in std_logic;
		en 			: in std_logic; 
		----------------------------------------------
		Imm_in		: in std_logic_vector(15 downto 0);
		OP_in		: in std_logic_vector(3 downto 0);
		DRA_in		: in std_logic_vector(15 downto 0);
		DRB_in		: in std_logic_vector(15 downto 0);
		ADW_in		: in std_logic_vector(2 downto 0);
		AOUT_in		: in std_logic_vector(15 downto 0);
		Flags_in	: in std_logic_vector(1 downto 0);
		Type_in		: in std_logic_vector(1 downto 0);
	
		PC_Inc_in	: in std_logic_vector(15 downto 0);
		PC_Imm_in   : in std_logic_vector(15 downto 0);
		---------------------------------------------
		Imm_out		: out std_logic_vector(15 downto 0);
		OP_out		: out std_logic_vector(3 downto 0);
		DRA_out		: out std_logic_vector(15 downto 0);
		DRB_out		: out std_logic_vector(15 downto 0);
		ADW_out		: out std_logic_vector(2 downto 0);
		AOUT_out	: out std_logic_vector(15 downto 0);
		Flags_out	: out std_logic_vector(1 downto 0);
		Type_out	: out std_logic_vector(1 downto 0);

		PC_Inc_out	: out std_logic_vector(15 downto 0);
		PC_Imm_out  : out std_logic_vector(15 downto 0);
		valid_in    : in  std_logic;
		valid_out	: out std_logic;
				LMSM_in			: in std_logic_vector( 1 downto 0);
		LMSM_out			: out std_logic_vector( 1 downto 0)
	
		);

end component;

component MEM_WB is
	port (
		clk			: in std_logic;
		clear		: in std_logic;
		reset       : in std_logic;
		en 			: in std_logic; 
		----------------------------------------------
		Imm_in		: in std_logic_vector(15 downto 0);
		OP_in		: in std_logic_vector(3 downto 0);
		ADW_in		: in std_logic_vector(2 downto 0);
		AOUT_in		: in std_logic_vector(15 downto 0);
		MOUT_in		: in std_logic_vector(15 downto 0);
		Flags_in	: in std_logic_vector(1 downto 0);
		Type_in		: in std_logic_vector(1 downto 0);

		PC_Inc_in	: in std_logic_vector(15 downto 0);
		PC_Imm_in   : in std_logic_vector(15 downto 0);
		---------------------------------------------
		
		Imm_out		: out std_logic_vector(15 downto 0);
		OP_out		: out std_logic_vector(3 downto 0);
		ADW_out		: out std_logic_vector(2 downto 0);
		AOUT_out	: out std_logic_vector(15 downto 0);
		MOUT_out	: out std_logic_vector(15 downto 0);
		Flags_out	: out std_logic_vector(1 downto 0);
		Type_out	: out std_logic_vector(1 downto 0);

		PC_Inc_out	: out std_logic_vector(15 downto 0);
		PC_Imm_out  : out std_logic_vector(15 downto 0)	;
		valid_in    : in  std_logic;
		valid_out	: out std_logic;
		LMSM_in			: in std_logic_vector( 1 downto 0);
		LMSM_out			: out std_logic_vector( 1 downto 0)
	
		);

end component;
---------------------------------------------------------------------



signal pc_out, im_ir, inc_if_id: std_logic_vector (15 downto 0) := (others => '0');

signal pc_pc_imm, ir_decoder, pc_inc_id, pc_imm_id, decoder_out,sign_ext_out, pc_id: std_logic_vector (15 downto 0) := (others => '0');

signal  pc_imm_rr, imm_rr, rf_d1_rr, rf_d2_rr, pc_inc_rr, rr_dra_out, mux_rr_imm_dout : std_logic_vector (15 downto 0) := (others => '0');

signal lm_sm_mux_rr_adw, adw_ex, adw_mem, arb_mux_rr, adw_rr, adw_wb, lm_sm_decoder_mux_addr2, lm_sm_addr3, mux_adw_out, decoder_mux_adw, ara_addr1_rr, arb_out, mux_out_addr2, reg_address_out : std_logic_vector(2 downto 0) := (others => '0'); 

signal lm_sm_rr, mux_lm_sm_out, lm_sm_reg_out, lm_sm_decode_dout : std_logic_vector(7 downto 0) := (others => '0');

signal op_ex, op_rr, op_mem, op_wb, op_reg_out : std_logic_vector(3 downto 0) := (others => '0');

signal wren_ex, pc_imm_ex, imm_ex, dra_ex,drb_ex,pc_inc_ex,aout_ex, mux_ex_alu1_out, mux_ex_alu2_out, mux_ex_pc_imm_out : std_logic_vector (15 downto 0) := (others => '0');

signal flags_mem1, flags_mem2,type_mem, type_rr, type_ex, flags_ex, flags_wb, reg_flag_wb_rf_out,mux_wb_rf_d3_control, type_wb : std_logic_vector(1 downto 0):= (others => '0');

signal ren_mem, imm_mem, dra_mem, drb_mem, aout_mem, pc_inc_mem,pc_imm_mem, mux_mem_addr_out, mux_mem_din_out, mem_dout : std_logic_vector (15 downto 0) := (others => '0');

signal imm_wb, dra_wb, drb_wb, aout_wb, mout_wb, pc_inc_wb, pc_imm_wb, mux_wb_rf_out, mux_wb_r7_out : std_logic_vector (15 downto 0) := (others => '0');
---enable signals
signal en_PC, r7_en, regfile_wr_en, en_rr_ex, en_ex_mem, en_mem_wb,reg_flag_wb_rf_en, lm_sm_main_enable_out : std_logic := '1';
---clear signals
signal clr_PC, clr_decoder, clr_if_id, clr_id_rr, clr_rr_ex, clr_ex_mem, clr_mem_wb,clr_reg_flag_wb_rf, wren_mem : std_logic := '0';

---control signals --muxes
signal  mux_rr_imm_control, mux_rr_addr2_rf_control, mux_rr_dra_control, mux_rr_adw_control, mux_ex_alu1_control, mux_mem_pc_control, mux_mem_addr_control, mux_mem_din_control, mux_wb_pc_control, mux_wb_r7_in_control : std_logic := '0'; 
signal mux_id_adw_control, reg_flag_wb_rf_d3_control, mux_ex_pc_imm_control, mux_ex_alu2_control : std_logic_vector(1 downto 0) := (others => '0'); --mux 4 to 1
---lm_sm_signals
signal lm_sm_mux_rr_dra, mux_wb_pc_out,mux_mem_pc_out  : std_logic_vector(15 downto 0) := (others => '0');

signal cmp_bit, valid, valid_reg_out, clr_lm_sm_reg, control_bit, valid_ex, valid_mem, valid_wb : std_logic :='0';

signal to_lm_sm_block, lm_sm_ex,lm_sm_mem,lm_sm_wb: std_logic_vector(1 downto 0) := (others => '0'); -- for lm_sm_decoder

begin

-----------------port mapping of control blocks path -----------------------------------
 RR_CONTROL : rr_mux_control
	port map(
	op 					=> op_rr,
	ara 				=> ara_addr1_rr,
	mux_rr_control 		=> mux_rr_imm_control
	);	

EX_CONTROL : controlblock_EX 
	port map(
		op 						=> op_ex,
		valid 					=> valid_ex,
		cmp 					=> cmp_bit,
		mux_ex_pc_imm_control 	=> mux_ex_pc_imm_control,
		mux_ex_alu1_control 	=> mux_ex_alu1_control,
		mux_ex_alu2_control 	=> mux_ex_alu2_control 
		);

MEM_CONTROL : controlblock_MEM 
  port map (
	valid 					=>valid_mem,
	flags_WB    			=> flags_mem1,
	op						=> op_mem,
	op_type					=> type_mem,
	Mux_MEM_ADDR_Control 	=> mux_mem_addr_control,
	Mux_MEM_DIN_Control 	=> mux_mem_din_control,
	Mux_MEM_PC_Control 		=> mux_mem_pc_control,
	Mem_wr_en			   => wren_mem,
	adw					   => adw_mem,
	LMSM						=> lm_sm_mem
  );	


WB_CONTROL_1 : wb_control 
port map(
	reg_flag_wb_rf 		   => reg_flag_wb_rf_out,
	op_type 			   => type_wb,
	op 					   => op_wb,
	mux_wb_pc_control 	   => mux_wb_pc_control,
	mux_wb_rf_d3_control   => mux_wb_rf_d3_control,
	wr_en 				   => regfile_wr_en,
	adw 				   => adw_wb,
	mux_r7_in_control      => mux_wb_r7_in_control,
	valid					=> valid,
	LMSM 					=> to_lm_sm_block
);




------------------port mapping of normaL DATAPATH components----------------------------



PC : my_reg 
	generic map (16)
	port map(
		clk 	=> clk, 	   
		en      => en_PC, 
		clr     => clr_PC,
		reset 	=> reset,
		din     => mux_wb_pc_out,
		dout    => pc_out		
		);

Instruction_memory :IM 
	PORT MAP(
		address		=> pc_out,		
		data_out	=> im_ir		
		);

Incrementor : increment 
port map(
 	din 	=> pc_out, 
 	dout	=> inc_if_id
 	);


IF_ID_REG : IF_ID 
	port map(     
		clk			=> clk,
		clr			=> clr_if_id,
		reset       => reset,
		en 			=> lm_sm_main_enable_out,

		pc_in 		=> pc_out,
		ir_in 		=> im_ir,
		pc_inc_in	=> inc_if_id,
	------------=> ,----------------
		pc_out 		=> pc_pc_imm,
		ir_out		=> decoder_out,
		pc_inc_out	=> pc_id
		);

sign_extendor : sign_ext

 port map(
 	imm6 	=> decoder_out(5 downto 0),
 	imm8	=> decoder_out(7 downto 0),
 	imm9	=> decoder_out(8 downto 0),
 	op_code => decoder_out(15 downto 12),
 	dout	=> sign_ext_out
 	);

PC_IMMIDIATE : adder 
   generic map(16)
   port map(
   		reg_a 	 => pc_pc_imm, 	
   		reg_b    => sign_ext_out,
        add_out  => pc_imm_id
        );


--DECODER : instr_decoder   
--  port map(
--  	   clock  	=> clk,
--  	   reset 	=> reset,ir
--       dout     => decoder_out
--      );

MUX_ID_ADW : mux4to1 
	generic map(3)
	port map(
		d0 		=> decoder_out(11 downto 9), 
		d1 		=> decoder_out(8 downto 6), 
	  	d2 		=> decoder_out(5 downto 3), 
	  	d3 		=> "000", 
	   sel		=>	mux_id_adw_control, 
		dout    => decoder_mux_adw
);
	------------------------------------------------- lm sm working----------------------------------------

mux_lm_sm :  mux2to1
	generic map(8)
	port map(
		d0 		=>  decoder_out(7 downto 0),
		d1  	=>  lm_sm_decode_dout,
	  	sel		=>  control_bit,
		dout 	=>  mux_lm_sm_out
	);	
lm_sm_reg : my_reg
	generic map(8)
	port map(
		clk 	=> clk,
		en 		=> '1',
		clr  	=> clr_lm_sm_reg,
		reset   => reset,
		din     => mux_lm_sm_out,
		dout    => lm_sm_reg_out
		);
op_reg_lm_sm : my_reg
	generic map(4)
	port map(
		clk 		=> clk,
		en 		=> '1',
		clr  		=> '0',
		reset   => reset,
		din     => decoder_out(15 downto 12),
		dout    => op_reg_out
		);

LM_SM_DECODE : lm_sm_decoder --addded
  port map (
  clk				=> clk,
	din    		=> lm_sm_reg_out,
	op				=> op_reg_out,
	LM_SM       => to_lm_sm_block,
    dout   		=> lm_sm_decode_dout,
    Reg_Address => reg_address_out,
	valid  		=> valid,
	control_bit => control_bit
  );


LM_SM_MAIN : LM_SM_Block 

	port map
	(
		clk		 				  => clk,  
		LMSM					  => to_lm_sm_block,  
		reset	 				  => reset,  
		valid					  => valid_reg_out,  
		mux_rr_adw_control		  => mux_rr_adw_control,  
		mux_rr_addr2_rf_control   => mux_rr_addr2_rf_control,  
		mux_rr_dra_control		  => mux_rr_dra_control,  
		enable				      => lm_sm_main_enable_out

	);
-------------------------------------------------------------------------
adw_control : id_adw_control
  port map (
	op	  				=> decoder_out(15 downto 12),
	id_adw_mux_control  => mux_id_adw_control
  ); 

------------------------------------------------------------------------

ID_RR_REG : ID_RR 
	port map(
		clk			=> clk,
		clr 		=> clr_id_rr,
		reset 		=> reset,
		en 			=> lm_sm_main_enable_out,
		pc_imm_in	=> pc_imm_id,
		imm_in  	=> sign_ext_out,
		ara_in		=> decoder_out(11 downto 9),
		arb_in		=> decoder_out(8 downto 6),
		adw_in		=> decoder_mux_adw,

		pc_inc_in 	=> pc_id,
		type_in		=> decoder_out(1 downto 0),

		valid_in	=> valid,
		valid_out   => valid_reg_out,
		--------------------------------------------
		pc_imm_out 	=> pc_imm_rr,
		imm_out   	=> imm_rr,
		ara_out		=> ara_addr1_rr,
		arb_out		=> arb_mux_rr,
		adw_out		=> adw_rr,

		pc_inc_out 	=> pc_inc_rr,
		type_out	=> type_rr,
	-----------------
		op_in		=> decoder_out(15 downto 12),
		op_out		=> op_rr
		);
	

 Mux_RR_Imm: mux2to1 
	generic map(16)
	port map(
		d0 		=>  inc_if_id,
		d1  	=>  imm_rr,
	  	sel		=>  mux_rr_imm_control,
		dout 	=>  mux_rr_imm_dout
	);
 
 Mux_RR_ADDR2_RF: mux2to1 
	generic map(3)
	port map(
		d0 		=> arb_mux_rr,
		d1  	=>  reg_address_out,
	  	sel		=>  mux_rr_addr2_rf_control,
		dout 	=> mux_out_addr2 
	);
 
 RR_REGFILE : regfile 
    port map (
    rf_d1          	=> rf_d1_rr,
    rf_d2          	=> rf_d2_rr,
    rf_d3          	=> mux_wb_rf_out,
	 R7_in			=> mux_wb_r7_out,
	 R7_enable		=> r7_en,
    wr_enable     	=> regfile_wr_en,
    Addr1   	    => ara_addr1_rr,
    Addr2	        => mux_out_addr2,
    Addr3   	    => adw_wb,
    clk 			=> clk,
	reset       	=> reset
    );

 Mux_RR_DRA: mux2to1 
	generic map(16)
	port map(
		d0 		=>  rf_d1_rr,
		d1  	=>  aout_ex,
	  	sel		=>  mux_rr_dra_control,
		dout 	=>  rr_dra_out
	);

 Mux_RR_ADW: mux2to1 
	generic map(3)
	port map(
		d0 		=>  adw_rr,
		d1  	=>  reg_address_out,
	  	sel		=>  mux_rr_adw_control,
		dout 	=>  mux_adw_out 
	);
 

RR_EX_REG: RR_EX
port map (
		clk			 => clk,
		clear		 => clr_rr_ex,
		reset        => reset,
		en 			 => en_rr_ex,

		PC_Imm_in 	 => pc_imm_rr,
		Imm_in 		 => imm_rr,
		OP_in 		 => op_rr,
		DRA_in 		 => rr_dra_out,
		DRB_in 		 => rf_d2_rr,
		ADW_in 		 => mux_adw_out,
		Type_in 	 => type_rr,

		PC_Inc_in 	 => pc_inc_rr,

		valid_in	=> valid_reg_out,
		valid_out   => valid_ex,		

		PC_Imm_out	 => pc_imm_ex,
		Imm_out 	 => imm_ex,
		OP_out 		 => op_ex,
		DRA_out 	 => dra_ex,
		DRB_out 	 => drb_ex,
		ADW_out 	 => adw_ex,
		Type_out 	 => type_ex,

		PC_Inc_out 	 => pc_inc_ex,
		LMSM_in			=> to_lm_sm_block,
		LMSM_out			=> lm_sm_ex
		);

---------------------------------------------------------
Mux_EX_ALU1: mux2to1 
	generic map(16)
	port map(
		d0 		=> imm_ex ,
		d1  	=>  dra_ex,
	  	sel		=>  mux_ex_alu1_control,
		dout 	=>  mux_ex_alu1_out
	);
 
Mux_EX_ALU2: mux4to1 
	generic map(16)
	port map(
		d0 		=>  drb_ex,
		d1  	=>  imm_ex,
		d2		=> "0000000000000001",
		d3		=> "0000000000000000",
	  	sel		=>  mux_ex_alu2_control,
		dout 	=>  mux_ex_alu2_out
	);
 
Mux_EX_PC_Imm: mux4to1 
	generic map(16)
	port map(
		d0 		=>  mux_rr_imm_dout,
		d1  	=>  pc_imm_ex,
		d2		=>  drb_ex,
		d3		=>  "0000000000000000",
	  	sel		=>  mux_ex_pc_imm_control,
		dout 	=>  mux_ex_pc_imm_out
	);
 
EX_ALU: ALU
   port map(
   	reg_a	=> mux_ex_alu1_out,
   	reg_b	=> mux_ex_alu2_out,
    OP 		=> op_ex,
    ALU_out => aout_ex,
    carry	=> flags_ex(1),
    zero	=> flags_ex(0),
	 cmp 	=> cmp_bit
	  );

EX_MEM_REG: EX_MEM 
	port map (
		clk			 => clk,
		clear		 => clr_ex_mem,
		reset 		 => reset,
		en 			 => en_ex_mem,
		------------ => ,
		Imm_in		 => imm_ex,
		OP_in		 => op_ex,
		DRA_in		 => dra_ex,
		DRB_in		 => drb_ex,
		ADW_in		 => adw_ex,
		AOUT_in		 => aout_ex,
		Flags_in	 => flags_ex,
		Type_in		 => type_ex,

		PC_Inc_in	 => pc_inc_ex,
		PC_Imm_in    => pc_imm_ex,

		valid_in	=> valid_ex,
		valid_out   => valid_mem,
		------------ => ,

		Imm_out		 => imm_mem,
		OP_out		 => op_mem,
		DRA_out		 => dra_mem,
		DRB_out		 => drb_mem,
		ADW_out		 => adw_mem,
		AOUT_out	 => aout_mem,
		Flags_out	 => flags_mem1, --  whcih are coming out of ex_mem
		Type_out	 => type_mem,

		PC_Inc_out	 => pc_inc_mem,
		PC_Imm_out   => pc_imm_mem,
		LMSM_in		=> lm_sm_ex,
		LMSM_out		=> lm_sm_mem
		);
	
Mux_MEM_PC : mux2to1
	generic map(16)
	port map(
		d0 		=>  mux_ex_pc_imm_out,
		d1  	=>  aout_mem,
	  	sel		=>  mux_mem_pc_control,
		dout 	=>  mux_mem_pc_out
	);

Mux_MEM_ADDR: mux2to1 
	generic map(16)
	port map(
		d0 		=>  aout_mem,
		d1  	=>  dra_mem,
	  	sel		=>  mux_mem_addr_control,
		dout 	=>  mux_mem_addr_out
	);

Mux_MEM_DIN: mux2to1 
	generic map(16)
	port map(
		d0 		=>  dra_mem,
		d1  	=>  drb_mem,
	  	sel		=>  mux_mem_din_control,
		dout 	=>  mux_mem_din_out
	);



DATA_MEMORY : MEM
	generic map (16,16)
	PORT map(
		clock				=> clk,
		data_in				=> mux_mem_din_out,
		address				=> mux_mem_addr_out,
		wr_en				=> wren_mem,
		data_out			=> mem_dout
	);

MEM_WB_REG: MEM_WB 
	port map 
	(
		clk			 => clk,
		clear		 => clr_mem_wb,
		reset 		 => reset,
		en 			 => en_mem_wb,
		------------ => ,
		Imm_in		 => imm_mem,
		OP_in		 => op_mem,
		ADW_in		 => adw_mem,
		AOUT_in		 => aout_mem,
		MOUT_in		 => mem_dout,
		Flags_in	 => flags_mem2, -- which are going in the next mem_wb stage 
		Type_in		 => type_mem,

		PC_Inc_in	 => pc_inc_mem,
		PC_Imm_in    => pc_imm_mem,
		------------ => ,
		Imm_out		 => imm_wb,
		OP_out		 => op_wb,
		ADW_out		 => adw_wb,
		AOUT_out	 => aout_wb,
		MOUT_out	 => mout_wb,
		Flags_out	 => flags_wb,
		Type_out	 => type_wb,

		PC_Inc_out	 => pc_inc_wb,
		PC_Imm_out   => pc_imm_wb,
			valid_in   => valid_mem,
			valid_out  => valid_wb,
				LMSM_in		=> lm_sm_mem,
		LMSM_out		=> lm_sm_wb
		);


MUX_WB_PC: mux2to1
	generic map(16)
	port map(
		d0 		=>  mux_mem_pc_out,
		d1  	=>  mout_wb,
	  	sel		=>  mux_wb_pc_control,
		dout 	=> mux_wb_pc_out 
	);

MUX_WB_RF_D3 : mux4to1 
	generic map(16)
	port map(
		d0 		=> imm_wb, 
		d1 		=> aout_wb, 
	  	d2 		=> mout_wb, 
	  	d3 		=> pc_inc_wb, 
	    sel		=> mux_wb_rf_d3_control, 
		dout    => mux_wb_rf_out 
);

MUX_WB_R7_IN: mux2to1
	generic map(16)
	port map(
		d0 		=> pc_inc_wb,
		d1  	=> pc_imm_wb,
	  	sel		=>  mux_wb_r7_in_control,
		dout 	=> mux_wb_r7_out 
	);

REG_FLAG_WB_RF: my_reg
	generic map (2)
	port map
	(
		clk 	=> clk,
		en 		=> reg_flag_wb_rf_en,
		clr  	=> clr_reg_flag_wb_rf,
		reset 	=> reset,
		din  	=> flags_wb,
		dout 	=> reg_flag_wb_rf_out
	);
--- i don't know whether it will work
zero_flag : process(mem_dout, op_mem, flags_mem1)
	begin
		if(mem_dout = "0000000000000000" and op_mem = "0100") then
			flags_mem2(0) <= '1'; --zero flag
			flags_mem2(1) <= flags_mem1(1); -- carry flag
		else 
			flags_mem2 <= flags_mem1;
		end if;	
	end process ; -- identifier	

-------asigning now may change later --------------------
en_PC 					<= '1';
r7_en 					<= '1';
en_rr_ex					<= '1';
en_ex_mem				<= '1';
en_mem_wb				<= '1';
reg_flag_wb_rf_en		<= '1';
clr_PC					<= '0';
clr_decoder				<= '0';
clr_if_id				<= '0';
clr_id_rr				<= '0';
clr_rr_ex				<= '0';
clr_ex_mem				<= '0';
clr_mem_wb				<= '0';
clr_reg_flag_wb_rf	<= '0';
clr_lm_sm_reg			<= '0';

----------------------------------------------------------	

-- LM_SM Block is missing. Needs to be entered
end arch;