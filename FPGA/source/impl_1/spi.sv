
module SpiFront(input  logic sck, 
               input  logic sdi,
               output logic sdo,
               input  logic done,
               output logic [511:0] key, plaintext,
               input  logic [511:0] cyphertext);

    logic         sdodelayed, wasdone;
    logic [511:0] cyphertextcaptured;
               
    // assert load
    // apply 256 sclks to shift in key and plaintext, starting with plaintext[127]
    // then deassert load, wait until done
    // then apply 128 sclks to shift out cyphertext, starting with cyphertext[127]
    // SPI mode is equivalent to cpol = 0, cpha = 0 since data is sampled on first edge and the first
    // edge is a rising edge (clock going from low in the idle state to high).
    always_ff @(posedge sck)
        if (!wasdone)  {cyphertextcaptured, plaintext, key} = {cyphertext, plaintext[510:0], key, sdi};
        else           {cyphertextcaptured, plaintext, key} = {cyphertextcaptured[510:0], plaintext, key, sdi}; 
    
    // sdo should change on the negative edge of sck
    always_ff @(negedge sck) begin
        wasdone = done;
        sdodelayed = cyphertextcaptured[510];
    end
    
    // when done is first asserted, shift out msb before clock edge
    assign sdo = (done & !wasdone) ? cyphertext[511] : sdodelayed;
endmodule
