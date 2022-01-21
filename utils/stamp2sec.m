function sec = stamp2sec(stamp)

    sec = double(stamp.Sec)+double(stamp.Nsec)*10^-9;

end