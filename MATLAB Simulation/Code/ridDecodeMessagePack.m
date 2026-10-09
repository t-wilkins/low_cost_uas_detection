function msgs = ridDecodeMessagePack(pack)
    % RIDDECODEMESSAGEPACK  Recover the N-by-singleSize message matrix from a
    % received Message Pack.
    
    pack = uint8(pack(:)).';
    if bitshift(pack(1), -4) ~= 15
        error('ridDecodeMessagePack:badType', ...
            'Not a Message Pack (type 0x%X).', bitshift(pack(1), -4));
    end
    
    singleSize = double(pack(2));
    n          = double(pack(3));
    body       = pack(4 : 3 + singleSize*n);
    msgs       = reshape(body, singleSize, n).';
end
