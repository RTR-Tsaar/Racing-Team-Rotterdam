def compare_all_ids(ids):
    # Convert decimal IDs to 11-bit binary strings
    ids_bin = [format(id, '011b') for id in ids]
    
    # Initialize the comparison result with all bits set to 'X'
    comparison_result = ['X'] * 11
    
    # Compare each bit position across all IDs
    for i in range(11):
        bit_set = {id_bin[i] for id_bin in ids_bin}
        if len(bit_set) == 1:
            comparison_result[i] = bit_set.pop()
    
    return ''.join(comparison_result)

def make_mask(result):
    # Initialize the variables
    bit_number = ['0'] * 11
    mask = ['0'] * 11
    
    # Iterate through the result to create bit_number and mask
    for i, bit in enumerate(result):
        if bit == '1':
            bit_number[i] = '1'
            mask[i] = '1'
        elif bit == '0':
            bit_number[i] = '0'
            mask[i] = '1'
        elif bit == 'X':
            bit_number[i] = '0'
            mask[i] = '0'
    
    # Convert lists to strings
    bit_number_str = ''.join(bit_number)
    mask_str = ''.join(mask)
    
    return bit_number_str, mask_str

# Example usage
ids = [1300, 1301, 1302, 1303, 1304, 850]
ids2 = [1000, 1001, 1002, 1003, 1004, 500]

result = compare_all_ids(ids)
result2 = compare_all_ids(ids2)

bit_number, mask = make_mask(result)
bit_number2, mask2 = make_mask(result2)

print(f"Comparison result for all IDs: {result}")
print(f"Bit number: 0b{bit_number}")
print(f"Mask: 0b{mask}")

print(f"Comparison result for all IDs 2: {result2}")
print(f"Bit number 2: 0b{bit_number2}")
print(f"Mask 2: 0b{mask2}")
