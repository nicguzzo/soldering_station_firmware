const R1:u32=10005;
const TH_TABLE:[(i32,i32);111]=[
(	6498	,	100	),
(	6698	,	99	),
(	6905	,	98	),
(	7120	,	97	),
(	7342	,	96	),
(	7573	,	95	),
(	7812	,	94	),
(	8060	,	93	),
(	8316	,	92	),
(	8583	,	91	),
(	8859	,	90	),
(	9146	,	89	),
(	9443	,	88	),
(	9751	,	87	),
(	10071	,	86	),
(	10404	,	85	),
(	10748	,	84	),
(	11106	,	83	),
(	11478	,	82	),
(	11864	,	81	),
(	12265	,	80	),
(	12681	,	79	),
(	13114	,	78	),
(	13564	,	77	),
(	14031	,	76	),
(	14517	,	75	),
(	15022	,	74	),
(	15548	,	73	),
(	16094	,	72	),
(	16662	,	71	),
(	17253	,	70	),
(	17869	,	69	),
(	18509	,	68	),
(	19175	,	67	),
(	19869	,	66	),
(	20592	,	65	),
(	21344	,	64	),
(	22127	,	63	),
(	22944	,	62	),
(	23794	,	61	),
(	24681	,	60	),
(	25605	,	59	),
(	26568	,	58	),
(	27573	,	57	),
(	28620	,	56	),
(	29713	,	55	),
(	30853	,	54	),
(	32043	,	53	),
(	33284	,	52	),
(	34580	,	51	),
(	35840	,	50	),
(	37346	,	49	),
(	38822	,	48	),
(	40364	,	47	),
(	41975	,	46	),
(	43659	,	45	),
(	45419	,	44	),
(	47259	,	43	),
(	49183	,	42	),
(	51195	,	41	),
(	53300	,	40	),
(	55503	,	39	),
(	57809	,	38	),
(	60222	,	37	),
(	62749	,	36	),
(	65395	,	35	),
(	68167	,	34	),
(	71072	,	33	),
(	74115	,	32	),
(	77305	,	31	),
(	80650	,	30	),
(	84157	,	29	),
(	87837	,	28	),
(	91697	,	27	),
(	95747	,	26	),
(	100000	,	25	),
(	104464	,	24	),
(	109152	,	23	),
(	114078	,	22	),
(	119253	,	21	),
(	124692	,	20	),
(	130410	,	19	),
(	136423	,	18	),
(	142748	,	17	),
(	149403	,	16	),
(	156407	,	15	),
(	163780	,	14	),
(	171545	,	13	),
(	179724	,	12	),
(	188343	,	11	),
(	198530	,	10	),
(	207005	,	9	),
(	217106	,	8	),
(	227764	,	7	),
(	239012	,	6	),
(	250886	,	5	),
(	263427	,	4	),
(	276676	,	3	),
(	290679	,	2	),
(	305482	,	1	),
(	321140	,	0	),
(	337705	,	-1	),
(	355239	,	-2	),
(	373806	,	-3	),
(	393473	,	-4	),
(	414316	,	-5	),
(	436413	,	-6	),
(	459851	,	-7	),
(	484723	,	-8	),
(	511127	,	-9	),
(	539171	,	-10	)
];

macro_rules! map_range { 
    ($a1:expr,$a2:expr,$b1:expr,$b2:expr,$s:expr)=>{
        ($b1 + ($s-$a1)*($b2-$b1)/($a2-$a1))
    }
}
const FP:i32=15;//fixed point at bit 15
const MASK:i32=(1<<FP)-1;

pub fn temperature(millivolts:u32,vdd:u32)-> f32 {
   let r2:i32 = ((millivolts*R1)/(vdd-millivolts)) as i32;   
   for (i,x) in TH_TABLE.iter().enumerate() {
        if x.0>=r2 {
            let j=if i>0 { i-1 }else { 0};
            let y=TH_TABLE[j];
            let temp=map_range!(x.0,y.0,x.1<<FP,y.1<<FP,r2);
            return temp as f32 /MASK as f32 ;
        }
   }
   0.0
}
