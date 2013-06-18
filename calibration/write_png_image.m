function [] = write_png_image(I, kk , calib_name , format_image , type_numbering , image_numbers , N_slots)


    format_image = 'png';



if ~type_numbering,   
    number_ext =  num2str(image_numbers(kk));
else
    number_ext = sprintf(['%.' num2str(N_slots) 'd'],image_numbers(kk));
end;

ima_name2 = [calib_name  number_ext '.' format_image];

fprintf(1,['Saving image under ' ima_name2 '...\n']);

imwrite(uint8(round(I)), ima_name2);
